// W03 GUI-free physics source using the unchanged W01 Go2Bridge.
// Accept the production simulator's -s/-i/-n contract so launch inspects real
// /proc arguments. Domain 41, lo and a dedicated discovery tag isolate all DDS.
// Gravity-free stationary qpos tests mapping connectivity, not locomotion.
#include "unitree_sdk2_bridge.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <fstream>
#include <sstream>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopped = 0;
void stop(int) {stopped = 1;}
}

int main(int argc, char ** argv)
{
  namespace po = boost::program_options;
  po::options_description options;
  options.add_options()
    ("scene,s", po::value<std::string>()->required())
    ("domain,i", po::value<int>()->required())
    ("interface,n", po::value<std::string>()->required())
    ("dds-config", po::value<std::string>()->required());
  po::variables_map args;
  po::store(po::parse_command_line(argc, argv, options), args);
  po::notify(args);
  if (args["domain"].as<int>() != 41 || args["interface"].as<std::string>() != "lo") {
    std::cerr << "W03 requires -i 41 -n lo\n";
    return 1;
  }
  std::ifstream config_file(args["dds-config"].as<std::string>());
  std::ostringstream config;
  config << config_file.rdbuf();
  if (config.str().find("<Tag>go2-w03-bringup-20261124</Tag>") == std::string::npos) {
    std::cerr << "W03 dedicated discovery tag required\n";
    return 1;
  }
  char error[1024]{};
  auto * model = mj_loadXML(args["scene"].as<std::string>().c_str(), nullptr, error, sizeof(error));
  if (!model) {std::cerr << error << '\n'; return 1;}
  auto * data = mj_makeData(model);
  model->opt.gravity[2] = 0;
  data->qpos[2] = 0.5;
  mj_forward(model, data);
  param::config.use_joystick = 0;
  param::config.print_scene_information = 0;
  unitree::common::JsonMap dds_config;
  dds_config["DomainId"] = 41;
  dds_config["Config"] = std::string(
    "<CycloneDDS><Domain id=\"41\"><General><Interfaces>"
    "<NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>,") +
    config.str();
  unitree::robot::ChannelFactory::Instance()->Init(dds_config);
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  {
    Go2Bridge bridge(model, data);
    const auto started = std::chrono::steady_clock::now();
    while (!stopped && data->time < 120.0) {
      mj_step(model, data);
      bridge.run();
      std::this_thread::sleep_until(
        started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(data->time)));
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
