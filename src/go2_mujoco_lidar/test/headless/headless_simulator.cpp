// GUI-free MuJoCo integration driver calling the W01-pinned Go2Bridge unchanged.
// Domain 1/lo only; no commands are published. Gravity-free motion provides base
// qpos ground truth and yawed translation; mj_resetData forces time backwards.
// This does not test RL/joystick locomotion or navigation performance.
#include "unitree_sdk2_bridge.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

int main(int argc, char ** argv)
{
  if (argc != 4 || !std::getenv("ROS_DOMAIN_ID") ||
    std::string(std::getenv("ROS_DOMAIN_ID")) != "1" ||
    !std::getenv("ROS_LOCALHOST_ONLY") ||
    std::string(std::getenv("ROS_LOCALHOST_ONLY")) != "1")
  {
    std::cerr << "requires model, ground-truth path, DDS config, Domain 1 and localhost only\n";
    return 1;
  }
  char error[1024]{};
  auto * model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {std::cerr << error << '\n'; return 1;}
  auto * data = mj_makeData(model);
  model->opt.gravity[2] = 0;
  param::config.use_joystick = 0;
  param::config.print_scene_information = 0;
  std::ifstream config_file(argv[3]);
  std::ostringstream config;
  config << config_file.rdbuf();
  if (config.str().find("<Tag>go2-w02-state-contract-20261124</Tag>") == std::string::npos)
  {
    std::cerr << "requires W02 discovery tag and loopback config\n";
    return 1;
  }
  unitree::common::JsonMap dds_config;
  dds_config["DomainId"] = 1;
  // ROS_LOCALHOST_ONLY configures RMW, not the native SDK participant.
  dds_config["Config"] = std::string(
    "<CycloneDDS><Domain id=\"1\"><General><Interfaces>"
    "<NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>,") +
    config.str();
  unitree::robot::ChannelFactory::Instance()->Init(dds_config);
  std::ofstream truth(argv[2]);
  truth << std::setprecision(17);
  truth << "epoch,stamp_ns,x,y,z,yaw,body_vx\n";
  {
    Go2Bridge bridge(model, data);
    // Allow DDS discovery while nodes are already running.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    for (int epoch = 0; epoch < 2; ++epoch) {
      mj_resetData(model, data);
      data->qpos[2] = 1;
      data->qpos[3] = std::sqrt(0.5);
      data->qpos[4] = 0; data->qpos[5] = 0;
      data->qpos[6] = (epoch == 0 ? 1 : -1) * std::sqrt(0.5);
      mj_forward(model, data);
      const auto started = std::chrono::steady_clock::now();
      const int steps = static_cast<int>(6.0 / model->opt.timestep);
      for (int step = 0; step < steps; ++step) {
        // First second stationary, then yawed body-forward translation at 0.2 m/s.
        const double speed = data->time >= 1.0 ? 0.2 : 0.0;
        data->qvel[0] = 0;
        data->qvel[1] = (epoch == 0 ? 1 : -1) * speed;
        mj_step(model, data);
        bridge.run();
        const auto stamp = std::llround(data->time * 1.0e9);
        truth << epoch << ',' << stamp << ',' << data->qpos[0] << ','
              << data->qpos[1] << ',' << data->qpos[2] << ','
              << (epoch == 0 ? 1 : -1) * std::acos(-1.0) / 2 << ',' << speed << '\n';
        std::this_thread::sleep_until(
          started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>((step + 1) * model->opt.timestep)));
      }
      truth.flush();
      std::cout << "EPOCH_DONE " << epoch << std::endl;
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  std::cout << "SIM_STOPPED" << std::endl;
  return 0;
}
