// W07 test-only native SDK discovery adapter. Original controller main/FSM/policy
// objects are linked unchanged. ChannelFactory's int/interface entry point builds
// its own XML; this wrapper supplies the same explicit XML used by our simulator.
// It requires a nonzero isolated domain (except reserved 42) on lo and never
// invokes the default initializer. W07 execution uses Domain 41.
#include <unitree/robot/channel/channel_factory.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

extern "C" void __wrap__ZN7unitree5robot14ChannelFactory4InitEiRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE(
  unitree::robot::ChannelFactory *factory, int32_t domain, const std::string &interface)
{
  const char *uri = std::getenv("CYCLONEDDS_URI");
  if ((domain <= 0 || domain == 42) || interface != "lo" || !uri ||
      std::string(uri).rfind("file://", 0) != 0)
    throw std::runtime_error("W07 RL discovery adapter requires Domain 41/lo/file XML");
  std::ifstream input(std::string(uri).substr(7));
  std::ostringstream config;
  config << input.rdbuf();
  if (config.str().find("<Tag>go2-w07-acceptance</Tag>") == std::string::npos)
    throw std::runtime_error("W07 discovery tag missing");
  unitree::common::JsonMap settings;
  settings["DomainId"] = domain;
  settings["Config"] = std::string(
    "<CycloneDDS><Domain id=\"" + std::to_string(domain) + "\"><General><Interfaces>"
    "<NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>,") +
    config.str();
  factory->Init(settings);
  std::cout << "W07_RL_EXPLICIT_DISCOVERY interface=lo" << std::endl;
}
