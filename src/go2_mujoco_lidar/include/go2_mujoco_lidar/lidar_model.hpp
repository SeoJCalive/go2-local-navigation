#pragma once
// MJCF raycast and IMU-site -> base state boundary. Raycast and odometry
// share the transformed base pose; twist uses the base axes and origin.

#include <mujoco/mujoco.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace go2_mujoco_lidar
{

struct BaseState
{
  std::array<double, 3> position;
  std::array<double, 4> quaternion;
  std::array<double, 3> linear_velocity;
  std::array<double, 3> angular_velocity;
};

BaseState base_state_from_imu(
  const std::array<double, 3> & position,
  const std::array<double, 4> & quaternion,
  const std::array<double, 3> & world_velocity,
  const std::array<double, 3> & body_gyro,
  const std::array<double, 3> & imu_offset);

struct RayHit
{
  float x;
  float y;
  float z;
  std::int32_t geom_id;
  std::int32_t body_id;
  bool robot_self;
  bool world_floor;
  std::int32_t ray_index;
};

std::vector<int> select_planar_ray_indices(
  const std::vector<std::array<double, 3>> & base_directions,
  int horizontal_samples);

std::vector<RayHit> select_planar_hits(
  const std::vector<RayHit> & hits,
  const std::vector<int> & selected_ray_indices);

class LidarModel
{
public:
  LidarModel(
    const std::string & model_path, int horizontal_samples, int vertical_samples,
    double vertical_min, double vertical_max, double range_min, double range_max,
    bool environment_only);

  void update_state(
    const BaseState & state,
    const std::array<double, 12> & motor_positions);
  const std::array<double, 3> & imu_offset() const {return imu_offset_;}
  std::vector<RayHit> cast() const;
  const std::vector<int> & planar_ray_indices() const;

private:
  struct ModelDeleter {void operator()(mjModel * value) const;};
  struct DataDeleter {void operator()(mjData * value) const;};

  bool is_robot_body(int body_id) const;
  bool is_world_floor_geom(int geom_id) const;
  void build_sensor_directions();
  void build_planar_ray_indices();

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  std::array<int, 12> motor_qpos_addresses_{};
  std::array<double, 3> imu_offset_{};
  std::vector<mjtNum> sensor_directions_;
  std::vector<int> planar_ray_indices_;
  std::array<mjtByte, mjNGROUP> ray_geom_groups_{};
  int base_body_id_{-1};
  int horizontal_samples_;
  int vertical_samples_;
  double vertical_min_;
  double vertical_max_;
  double range_min_;
  double range_max_;
  bool environment_only_;
};

}
