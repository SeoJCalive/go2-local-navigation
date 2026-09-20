#include "go2_mujoco_lidar/lidar_model.hpp"

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
  const unitree_go::msg::SportModeState & sport_state)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = kOdomFrame;
  odometry.child_frame_id = kBaseFrame;
  odometry.pose.pose.position.x = sport_state.position[0];
  odometry.pose.pose.position.y = sport_state.position[1];
  odometry.pose.pose.position.z = sport_state.position[2];
  odometry.pose.pose.orientation.w = sport_state.imu_state.quaternion[0];
  odometry.pose.pose.orientation.x = sport_state.imu_state.quaternion[1];
  odometry.pose.pose.orientation.y = sport_state.imu_state.quaternion[2];
  odometry.pose.pose.orientation.z = sport_state.imu_state.quaternion[3];
  odometry.twist.twist.linear.x = sport_state.velocity[0];
  odometry.twist.twist.linear.y = sport_state.velocity[1];
  odometry.twist.twist.linear.z = sport_state.velocity[2];
  odometry.twist.twist.angular.x = sport_state.imu_state.gyroscope[0];
  odometry.twist.twist.angular.y = sport_state.imu_state.gyroscope[1];
  odometry.twist.twist.angular.z = sport_state.imu_state.gyroscope[2];
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
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);
    maximum_state_skew_milliseconds_ =
      declare_parameter<std::int64_t>("maximum_state_skew_milliseconds", 2);
    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    if (maximum_state_skew_milliseconds_ < 0) {
      throw std::invalid_argument("maximum_state_skew_milliseconds must be non-negative");
    }
    const auto subscription_qos = rclcpp::SensorDataQoS();
    const auto publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    raw_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kRawTopic, publisher_qos);
    filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kFilteredTopic, publisher_qos);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      kOdomTopic, publisher_qos);
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    low_state_subscription_ = create_subscription<unitree_go::msg::LowState>(
      declare_parameter<std::string>("low_state_topic", "/lowstate"), subscription_qos,
      [this](unitree_go::msg::LowState::ConstSharedPtr message) {
        low_state_ = std::move(message);
      });
    sport_state_subscription_ = create_subscription<unitree_go::msg::SportModeState>(
      declare_parameter<std::string>("sport_state_topic", "/sportmodestate"), subscription_qos,
      [this](unitree_go::msg::SportModeState::ConstSharedPtr message) {
        sport_state_ = std::move(message);
        rosgraph_msgs::msg::Clock clock;
        clock.clock.sec = sport_state_->stamp.sec;
        clock.clock.nanosec = sport_state_->stamp.nanosec;
        clock_publisher_->publish(clock);
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&MujocoLidarNode::publish_scan, this));
    RCLCPP_INFO(get_logger(), "waiting for /lowstate and /sportmodestate before publishing");
  }

private:
  void publish_scan()
  {
    if (!low_state_ || !sport_state_) {
      return;
    }
    const std::int64_t sport_milliseconds =
      static_cast<std::int64_t>(sport_state_->stamp.sec) * 1000 +
      static_cast<std::int64_t>(sport_state_->stamp.nanosec) / 1000000;
    const auto state_skew_milliseconds = std::llabs(
      sport_milliseconds - static_cast<std::int64_t>(low_state_->tick));
    if (state_skew_milliseconds > maximum_state_skew_milliseconds_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "state timestamps differ by %ld ms; dropping LiDAR frame",
        static_cast<long>(state_skew_milliseconds));
      return;
    }
    std::array<double, 3> imu_position{};
    std::array<double, 4> quaternion{};
    std::array<double, 12> motors{};
    for (std::size_t index = 0; index < imu_position.size(); ++index) {
      imu_position[index] = sport_state_->position[index];
    }
    for (std::size_t index = 0; index < quaternion.size(); ++index) {
      quaternion[index] = sport_state_->imu_state.quaternion[index];
    }
    for (std::size_t index = 0; index < motors.size(); ++index) {
      motors[index] = low_state_->motor_state[index].q;
    }
    try {
      model_.update_state(imu_position, quaternion, motors);
      const auto hits = model_.cast();
      std::size_t self_hits = 0;
      for (const auto & hit : hits) {
        self_hits += hit.robot_self ? 1U : 0U;
      }
      const rclcpp::Time stamp(
        sport_state_->stamp.sec, sport_state_->stamp.nanosec, RCL_ROS_TIME);
      raw_publisher_->publish(make_cloud(stamp, hits, false));
      filtered_publisher_->publish(make_cloud(stamp, hits, true));
      odometry_publisher_->publish(make_odometry(stamp, *sport_state_));
      if (!first_scan_logged_) {
        RCLCPP_INFO(
          get_logger(), "first scan: raw=%zu self=%zu filtered=%zu",
          hits.size(), self_hits, hits.size() - self_hits);
        first_scan_logged_ = true;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  LidarModel model_;
  bool first_scan_logged_{false};
  std::int64_t maximum_state_skew_milliseconds_{2};
  unitree_go::msg::LowState::ConstSharedPtr low_state_;
  unitree_go::msg::SportModeState::ConstSharedPtr sport_state_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
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
