// W07 gravity-enabled physics with unchanged W01 Go2Bridge and real RL/FIFO.
// The -s/-i/-n contract lets the session inspect world/domain through /proc.
// truth.csv records base freejoint qpos independently of ROS sensor conversion.
// A dedicated reset request file triggers mj_resetData without node restarts.
#include "unitree_sdk2_bridge.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

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
    ("output", po::value<std::string>()->required());
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
  {
    Go2Bridge bridge(model, data);
    auto started = std::chrono::steady_clock::now();
    int epoch = 0;
    long steps = 0;
    while (!stopped) {
      if (std::filesystem::exists(output / "reset.request")) {
        std::filesystem::remove(output / "reset.request");
        mj_resetData(model, data);
        mj_forward(model, data);
        started = std::chrono::steady_clock::now();
        ++epoch;
        std::cout << "SIM_RESET epoch=" << epoch << std::endl;
      }
      mj_step(model, data);
      bridge.run();
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
