// W07 gravity-enabled physics with unchanged W01 Go2Bridge and real RL/FIFO.
// The -s/-i/-n contract lets the session inspect world/domain through /proc.
// truth.csv records base freejoint qpos independently of ROS sensor conversion.
// A dedicated reset request file triggers mj_resetData without node restarts.
// contacts.csv records every robot/world non-plane contact at each
// physics step, including contacts between the 20 Hz truth samples.
#include "unitree_sdk2_bridge.h"
#include "simulation_step.hpp"
#include "../wall_hug/clearance_2d.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) {stopped = 1;}
}

int main(int argc, char **argv)
{
  namespace po = boost::program_options;
  po::options_description options;
  options.add_options()
    ("scene,s", po::value<std::string>()->required())
    ("domain,i", po::value<int>()->required())
    ("interface,n", po::value<std::string>()->required())
    ("dds-config", po::value<std::string>()->required())
    ("fifo", po::value<std::string>()->required())
    ("output", po::value<std::string>()->required())
    ("progress-log", po::value<std::string>());
  po::variables_map args;
  po::store(po::parse_command_line(argc, argv, options), args);
  po::notify(args);
  if ((args["domain"].as<int>() <= 0 || args["domain"].as<int>() == 42) || args["interface"].as<std::string>() != "lo")
    return 1;
  std::ifstream config_file(args["dds-config"].as<std::string>());
  std::ostringstream config;
  config << config_file.rdbuf();
  if (config.str().find("<Tag>go2-w07-acceptance</Tag>") == std::string::npos)
    return 1;
  char error[1024]{};
  auto *model = mj_loadXML(args["scene"].as<std::string>().c_str(), nullptr, error, sizeof(error));
  if (!model) {std::cerr << error << '\n'; return 1;}
  auto *data = mj_makeData(model);
  mj_forward(model, data);
  param::config.use_joystick = 1;
  param::config.joystick_type = "switch";
  param::config.joystick_device = args["fifo"].as<std::string>();
  param::config.joystick_bits = 16;
  param::config.print_scene_information = 0;
  unitree::common::JsonMap dds_config;
  dds_config["DomainId"] = args["domain"].as<int>();
  dds_config["Config"] = std::string(
    "<CycloneDDS><Domain id=\"" + std::to_string(args["domain"].as<int>()) + "\"><General><Interfaces>"
    "<NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>,") +
    config.str();
  unitree::robot::ChannelFactory::Instance()->Init(dds_config);
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  const auto output = std::filesystem::path(args["output"].as<std::string>());
  std::ofstream truth(output / "truth.csv");
  truth << "epoch,sim_time,x,y,z,qw,qx,qy,qz\n" << std::setprecision(17);
  std::ofstream contacts(output / "contacts.csv");
  contacts << "epoch,sim_time,robot_geom,world_geom,distance\n" << std::setprecision(17);
  // G03 observes every step without changing physics, collision masks or policy.
  // Reuse G02's conservative horizontal support envelopes, including moving legs.
  std::ofstream physics, progress;
  std::vector<int> obstacles, robot_geoms;
  bool contact_alert = false, fall_alert = false;
  if (args.count("progress-log")) {
    progress.open(args["progress-log"].as<std::string>(), std::ios::app);
    physics.open(output / "physics.csv");
    physics << "sim_time,x,y,z,min_clearance_m,robot_geom,world_geom,fall\n"
            << std::setprecision(17);
    for (int g = 0; g < model->ngeom; ++g) {
      if (!model->geom_contype[g] && !model->geom_conaffinity[g]) continue;
      if (model->geom_bodyid[g] != 0) robot_geoms.push_back(g);
      else if (model->geom_type[g] != mjGEOM_PLANE) {
        if (model->geom_type[g] != mjGEOM_BOX && model->geom_type[g] != mjGEOM_CYLINDER)
          throw std::runtime_error("G03 world envelope supports only box/cylinder");
        obstacles.push_back(g);
      }
    }
    if (obstacles.empty() || robot_geoms.empty())
      throw std::runtime_error("G03 collision envelope geometry missing");
  }
  {
    Go2Bridge bridge(model, data);
    auto started = std::chrono::steady_clock::now();
    int epoch = 0;
    long steps = 0;
    std::uint64_t epoch_steps = 0;
    while (!stopped) {
      if (std::filesystem::exists(output / "reset.request")) {
        std::filesystem::remove(output / "reset.request");
        mj_resetData(model, data);
        mj_forward(model, data);
        started = std::chrono::steady_clock::now();
        ++epoch;
        epoch_steps = 0;
        std::cout << "SIM_RESET epoch=" << epoch << std::endl;
      }
      step_epoch(model, data, epoch_steps);
      bridge.run();
      for (int i = 0; i < data->ncon; ++i) {
        const auto &contact = data->contact[i];
        const int a = contact.geom[0], b = contact.geom[1];
        const bool a_world = model->geom_bodyid[a] == 0;
        const bool b_world = model->geom_bodyid[b] == 0;
        if (a_world == b_world) continue;
        const int obstacle = a_world ? a : b, robot = a_world ? b : a;
        if (model->geom_type[obstacle] == mjGEOM_PLANE) continue;
        contacts << epoch << ',' << data->time << ',' << robot << ','
                 << mj_id2name(model, mjOBJ_GEOM, obstacle) << ','
                 << contact.dist << std::endl;
        if (progress.is_open() && !contact_alert) {
          progress << "G03/경보 비바닥 접촉 sim=" << data->time << std::endl;
          contact_alert = true;
        }
      }
      if (physics.is_open()) {
        double minimum = std::numeric_limits<double>::infinity();
        int nearest_robot = -1, nearest_world = -1;
        for (int w : obstacles) for (int r : robot_geoms) {
          const double gap = clearance2d(model, data, w, r);
          if (gap < minimum) {minimum = gap; nearest_robot = r; nearest_world = w;}
        }
        const auto* q = data->qpos + 3;
        const double roll = std::atan2(2*(q[0]*q[1]+q[2]*q[3]),
                                      1-2*(q[1]*q[1]+q[2]*q[2]));
        const double pitch = std::asin(std::clamp(2*(q[0]*q[2]-q[3]*q[1]), -1., 1.));
        const bool fall = data->qpos[2] < .18 || std::max(std::abs(roll), std::abs(pitch)) > M_PI/3;
        physics << data->time << ',' << data->qpos[0] << ',' << data->qpos[1] << ','
                << data->qpos[2] << ',' << minimum << ',' << nearest_robot << ','
                << nearest_world << ',' << fall << '\n';
        if (fall && !fall_alert) {
          progress << "G03/경보 낙상 sim=" << data->time << std::endl;
          fall_alert = true;
        }
        if (steps % 25 == 0) physics.flush();
      }
      if (++steps % 25 == 0) {
        truth << epoch << ',' << data->time;
        for (int i = 0; i < 7; ++i) truth << ',' << data->qpos[i];
        truth << std::endl;
      }
      std::this_thread::sleep_until(
        started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(data->time)));
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}
