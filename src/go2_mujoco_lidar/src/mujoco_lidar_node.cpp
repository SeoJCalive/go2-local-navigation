// Join simulation-only DDS state at identical stamps, then convert to base
// pose/body twist. The wall timer consumes each fresh pair once for cloud/odom;
// Sport input drives /clock. Reset clears the epoch; steady time detects stalls.
// Consecutive state/raycast failures exhaust a positive error budget and escape
// the executor to main's non-zero exit; a successful scan clears that budget.
// This node has no physical command interface.
#include "go2_mujoco_lidar/lidar_model.hpp"
#include "go2_mujoco_lidar/step_pairing.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace go2_mujoco_lidar
{
namespace
{
constexpr char kRawTopic[] = "/utlidar/cloud";
constexpr char kFilteredTopic[] = "/simulation/utlidar/cloud_self_filtered";
constexpr char kPlanarTopic[] = "/simulation/utlidar/cloud_planar_selected";
constexpr char kOdomTopic[] = "/utlidar/robot_odom";
constexpr char kFrame[] = "utlidar_lidar";
constexpr char kOdomFrame[] = "odom";
constexpr char kBaseFrame[] = "base_link";

sensor_msgs::msg::PointCloud2 make_cloud(
  const rclcpp::Time & stamp, const std::vector<RayHit> & hits, bool remove_self)
{
  std::size_t output_size = 0;
  for (const auto & hit : hits) {
    output_size += (!remove_self || !hit.robot_self) ? 1U : 0U;
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = kFrame;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "hit_geom_id", 1, sensor_msgs::msg::PointField::INT32,
    "hit_body_id", 1, sensor_msgs::msg::PointField::INT32);
  modifier.resize(output_size);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<std::int32_t> geom(cloud, "hit_geom_id");
  sensor_msgs::PointCloud2Iterator<std::int32_t> body(cloud, "hit_body_id");
  for (const auto & hit : hits) {
    if (remove_self && hit.robot_self) {
      continue;
    }
    *x = hit.x; *y = hit.y; *z = hit.z;
    *geom = hit.geom_id; *body = hit.body_id;
    ++x; ++y; ++z; ++geom; ++body;
  }
  cloud.is_dense = true;
  return cloud;
}

nav_msgs::msg::Odometry make_odometry(
  const rclcpp::Time & stamp,
  const BaseState & state)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = kOdomFrame;
  odometry.child_frame_id = kBaseFrame;
  odometry.pose.pose.position.x = state.position[0];
  odometry.pose.pose.position.y = state.position[1];
  odometry.pose.pose.position.z = state.position[2];
  odometry.pose.pose.orientation.w = state.quaternion[0];
  odometry.pose.pose.orientation.x = state.quaternion[1];
  odometry.pose.pose.orientation.y = state.quaternion[2];
  odometry.pose.pose.orientation.z = state.quaternion[3];
  odometry.twist.twist.linear.x = state.linear_velocity[0];
  odometry.twist.twist.linear.y = state.linear_velocity[1];
  odometry.twist.twist.linear.z = state.linear_velocity[2];
  odometry.twist.twist.angular.x = state.angular_velocity[0];
  odometry.twist.twist.angular.y = state.angular_velocity[1];
  odometry.twist.twist.angular.z = state.angular_velocity[2];
  return odometry;
}
}

class MujocoLidarNode final : public rclcpp::Node
{
public:
  MujocoLidarNode()
  : Node("go2_mujoco_lidar"),
    model_(
      declare_parameter<std::string>("model_path"),
      declare_parameter<int>("horizontal_samples", 360),
      declare_parameter<int>("vertical_samples", 8),
      declare_parameter<double>("vertical_min", -0.2617993877991494),
      declare_parameter<double>("vertical_max", 0.2617993877991494),
      declare_parameter<double>("range_min", 0.25),
      declare_parameter<double>("range_max", 5.0),
      declare_parameter<bool>("environment_only", false))
  {
    raycast_error_limit_ = declare_parameter<int>("raycast_error_limit", 3);
    if (raycast_error_limit_ <= 0) {
      throw std::invalid_argument("raycast_error_limit must be positive");
    }
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);
    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    const auto subscription_qos = rclcpp::SensorDataQoS();
    const auto publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    raw_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kRawTopic, publisher_qos);
    filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kFilteredTopic, publisher_qos);
    planar_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kPlanarTopic, publisher_qos);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      kOdomTopic, publisher_qos);
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    low_state_subscription_ = create_subscription<unitree_go::msg::LowState>(
      declare_parameter<std::string>("low_state_topic", "/lowstate"), subscription_qos,
      [this](unitree_go::msg::LowState::ConstSharedPtr message) {
        const auto stamp = static_cast<std::int64_t>(message->tick) * 1000000;
        if (pairing_.add_low(stamp, std::move(message))) {
          log_reset();
        }
      });
    sport_state_subscription_ = create_subscription<unitree_go::msg::SportModeState>(
      declare_parameter<std::string>("sport_state_topic", "/sportmodestate"), subscription_qos,
      [this](unitree_go::msg::SportModeState::ConstSharedPtr message) {
        const auto stamp = static_cast<std::int64_t>(message->stamp.sec) * 1000000000 +
          message->stamp.nanosec;
        const auto sec = message->stamp.sec;
        const auto nanosec = message->stamp.nanosec;
        if (pairing_.add_sport(stamp, std::move(message))) {
          log_reset();
        }
        rosgraph_msgs::msg::Clock clock;
        clock.clock.sec = sec;
        clock.clock.nanosec = nanosec;
        clock_publisher_->publish(clock);
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&MujocoLidarNode::publish_scan, this));
    RCLCPP_INFO(get_logger(), "waiting for /lowstate and /sportmodestate before publishing");
  }

private:
  void log_reset()
  {
    RCLCPP_WARN(get_logger(), "simulation time moved backwards; resetting paired state epoch");
    last_fresh_input_ = std::chrono::steady_clock::now();
    stale_logged_ = false;
  }

  void publish_scan()
  {
    const auto pair = pairing_.take();
    if (!pair) {
      if (!stale_logged_ &&
        std::chrono::steady_clock::now() - last_fresh_input_ > std::chrono::seconds(1))
      {
        RCLCPP_WARN(
          get_logger(), "no fresh matched simulation step for 1 s; output stopped "
          "(matched=%zu buffer_dropped=%zu)",
          pairing_.matched(), pairing_.dropped());
        stale_logged_ = true;
      }
      return;
    }
    last_fresh_input_ = std::chrono::steady_clock::now();
    if (stale_logged_) {
      RCLCPP_INFO(get_logger(), "fresh matched simulation input recovered");
      stale_logged_ = false;
    }
    const auto & sport_state = *pair->sport;
    std::array<double, 3> imu_position{};
    std::array<double, 4> quaternion{};
    std::array<double, 3> world_velocity{};
    std::array<double, 3> body_gyro{};
    std::array<double, 12> motors{};
    for (std::size_t index = 0; index < imu_position.size(); ++index) {
      imu_position[index] = sport_state.position[index];
      world_velocity[index] = sport_state.velocity[index];
      body_gyro[index] = sport_state.imu_state.gyroscope[index];
    }
    for (std::size_t index = 0; index < quaternion.size(); ++index) {
      quaternion[index] = sport_state.imu_state.quaternion[index];
    }
    for (std::size_t index = 0; index < motors.size(); ++index) {
      motors[index] = pair->low->motor_state[index].q;
    }
    try {
      const auto state = base_state_from_imu(
        imu_position, quaternion, world_velocity, body_gyro, model_.imu_offset());
      model_.update_state(state, motors);
      const auto hits = model_.cast();
      const auto planar_hits = select_planar_hits(hits, model_.planar_ray_indices());
      std::size_t self_hits = 0;
      for (const auto & hit : hits) {
        self_hits += hit.robot_self ? 1U : 0U;
      }
      const rclcpp::Time stamp(
        sport_state.stamp.sec, sport_state.stamp.nanosec, RCL_ROS_TIME);
      raw_publisher_->publish(make_cloud(stamp, hits, false));
      filtered_publisher_->publish(make_cloud(stamp, hits, true));
      planar_publisher_->publish(make_cloud(stamp, planar_hits, false));
      odometry_publisher_->publish(make_odometry(stamp, state));
      consecutive_raycast_errors_ = 0;
      if (!first_scan_logged_) {
        RCLCPP_INFO(
          get_logger(), "first scan: raw=%zu self=%zu filtered=%zu planar=%zu",
          hits.size(), self_hits, hits.size() - self_hits, planar_hits.size());
        first_scan_logged_ = true;
      }
    } catch (const std::exception & error) {
      ++consecutive_raycast_errors_;
      RCLCPP_ERROR(
        get_logger(), "raycast error %d/%d: %s",
        consecutive_raycast_errors_, raycast_error_limit_, error.what());
      if (consecutive_raycast_errors_ >= raycast_error_limit_) {
        throw std::runtime_error("consecutive raycast error budget exhausted");
      }
    }
  }

  LidarModel model_;
  int raycast_error_limit_{3};
  int consecutive_raycast_errors_{0};
  bool first_scan_logged_{false};
  bool stale_logged_{false};
  std::chrono::steady_clock::time_point last_fresh_input_{std::chrono::steady_clock::now()};
  StepPairing<unitree_go::msg::LowState, unitree_go::msg::SportModeState> pairing_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planar_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_subscription_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_mujoco_lidar::MujocoLidarNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_mujoco_lidar"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
