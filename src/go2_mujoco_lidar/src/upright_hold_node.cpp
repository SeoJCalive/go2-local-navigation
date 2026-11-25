#include <motor_crc.h>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_cmd.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr char kLowCmdTopic[] = "/lowcmd";
constexpr double kControlPeriodSeconds = 0.002;
constexpr double kRampSeconds = 1.2;
constexpr double kRampEndSeconds = 3.0;
constexpr std::array<double, 12> kStandUpJointPositions{
  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
constexpr std::array<double, 12> kStandDownJointPositions{
  0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
  0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};

std::string cyclone_configuration()
{
  const char * const uri = std::getenv("CYCLONEDDS_URI");
  if (uri == nullptr) {
    return "";
  }
  const std::string value{uri};
  constexpr char kFileUriPrefix[] = "file://";
  if (value.rfind(kFileUriPrefix, 0) != 0) {
    return value;
  }
  std::ifstream configuration(value.substr(std::char_traits<char>::length(kFileUriPrefix)));
  return {std::istreambuf_iterator<char>(configuration), std::istreambuf_iterator<char>()};
}

void require_simulation_environment()
{
  const char * const domain = std::getenv("ROS_DOMAIN_ID");
  if (domain == nullptr || std::string{domain} != "1") {
    throw std::runtime_error("simulation upright hold requires ROS_DOMAIN_ID=1");
  }
  const std::string configuration = cyclone_configuration();
  if (configuration.find("name=\"lo\"") == std::string::npos ||
    configuration.find("127.0.0.1") == std::string::npos)
  {
    throw std::runtime_error("simulation upright hold requires CycloneDDS loopback lo and 127.0.0.1");
  }
}
}

class UprightHoldNode final : public rclcpp::Node
{
public:
  UprightHoldNode()
  : Node("go2_mujoco_upright_hold")
  {
    require_simulation_environment();
    publisher_ = create_publisher<unitree_go::msg::LowCmd>(kLowCmdTopic, 10);
    initialize_command();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(kControlPeriodSeconds),
      std::bind(&UprightHoldNode::publish_hold, this));
    RCLCPP_INFO(
      get_logger(), "simulation-only command owner: node=go2_mujoco_upright_hold topic=/lowcmd");
  }

private:
  void initialize_command()
  {
    for (auto & motor : command_.motor_cmd) {
      motor.mode = 0x01;
      motor.q = PosStopF;
      motor.kp = 0.0;
      motor.dq = VelStopF;
      motor.kd = 0.0;
      motor.tau = 0.0;
    }
  }

  void publish_hold()
  {
    elapsed_seconds_ += kControlPeriodSeconds;
    const double phase = std::tanh(elapsed_seconds_ / kRampSeconds);
    for (std::size_t index = 0; index < kStandUpJointPositions.size(); ++index) {
      auto & motor = command_.motor_cmd[index];
      motor.q = elapsed_seconds_ < kRampEndSeconds ?
        phase * kStandUpJointPositions[index] + (1.0 - phase) * kStandDownJointPositions[index] :
        kStandUpJointPositions[index];
      motor.dq = 0.0;
      motor.kp = elapsed_seconds_ < kRampEndSeconds ? phase * 50.0 + (1.0 - phase) * 20.0 : 50.0;
      motor.kd = 3.5;
      motor.tau = 0.0;
    }
    get_crc(command_);
    publisher_->publish(command_);
  }

  double elapsed_seconds_{0.0};
  unitree_go::msg::LowCmd command_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UprightHoldNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_mujoco_upright_hold"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
