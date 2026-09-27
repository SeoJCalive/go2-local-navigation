#include "go2_mujoco_lidar/lidar_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_mujoco_lidar
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, 3> kImuOffset{-0.02557, 0.0, 0.04232};
constexpr std::array<double, 3> kLidarTranslation{0.28945, 0.0, -0.046825};
constexpr double kLidarPitch = 2.8782;

const std::array<const char *, 12> kMotorJointNames{
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

std::array<double, 3> rotate(
  const std::array<double, 4> & quaternion, const std::array<double, 3> & point)
{
  mjtNum output[3]{};
  const mjtNum input[3]{point[0], point[1], point[2]};
  const mjtNum rotation[4]{quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
  mju_rotVecQuat(output, input, rotation);
  return {output[0], output[1], output[2]};
}

std::array<double, 4> normalized(std::array<double, 4> quaternion)
{
  const double norm = std::sqrt(
    quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
    quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  if (norm < 1.0e-9) {
    throw std::runtime_error("base quaternion is zero");
  }
  for (double & value : quaternion) {
    value /= norm;
  }
  return quaternion;
}
}

std::vector<int> select_planar_ray_indices(
  const std::vector<std::array<double, 3>> & base_directions,
  int horizontal_samples)
{
  if (horizontal_samples < 1) {
    throw std::invalid_argument("horizontal_samples must be positive");
  }
  std::vector<int> selected(horizontal_samples, -1);
  std::vector<double> best_elevation(horizontal_samples, std::numeric_limits<double>::infinity());
  for (std::size_t ray = 0; ray < base_directions.size(); ++ray) {
    const auto & direction = base_directions[ray];
    const double azimuth = std::atan2(direction[1], direction[0]);
    const double planar_length = std::hypot(direction[0], direction[1]);
    const double elevation = std::abs(std::atan2(direction[2], planar_length));
    const int bin = std::min(
      horizontal_samples - 1,
      static_cast<int>((azimuth + kPi) * horizontal_samples / (2.0 * kPi)));
    constexpr double tie_tolerance = 1.0e-12;
    if (elevation + tie_tolerance < best_elevation[bin] ||
      (std::abs(elevation - best_elevation[bin]) <= tie_tolerance &&
      (selected[bin] < 0 || static_cast<int>(ray) < selected[bin])))
    {
      selected[bin] = static_cast<int>(ray);
      best_elevation[bin] = elevation;
    }
  }
  return selected;
}

std::vector<RayHit> select_planar_hits(
  const std::vector<RayHit> & hits,
  const std::vector<int> & selected_ray_indices)
{
  int maximum_index = -1;
  for (const int index : selected_ray_indices) {
    maximum_index = std::max(maximum_index, index);
  }
  std::vector<bool> selected(static_cast<std::size_t>(maximum_index + 1), false);
  for (const int index : selected_ray_indices) {
    if (index >= 0) {
      selected[static_cast<std::size_t>(index)] = true;
    }
  }
  std::vector<RayHit> output;
  output.reserve(selected_ray_indices.size());
  for (const auto & hit : hits) {
    if (!hit.robot_self && !hit.world_floor && hit.ray_index >= 0 &&
      static_cast<std::size_t>(hit.ray_index) < selected.size() &&
      selected[static_cast<std::size_t>(hit.ray_index)])
    {
      output.push_back(hit);
    }
  }
  return output;
}

void LidarModel::ModelDeleter::operator()(mjModel * value) const {mj_deleteModel(value);}
void LidarModel::DataDeleter::operator()(mjData * value) const {mj_deleteData(value);}

LidarModel::LidarModel(
  const std::string & model_path, int horizontal_samples, int vertical_samples,
  double vertical_min, double vertical_max, double range_min, double range_max,
  bool environment_only)
: horizontal_samples_(horizontal_samples), vertical_samples_(vertical_samples),
  vertical_min_(vertical_min), vertical_max_(vertical_max),
  range_min_(range_min), range_max_(range_max), environment_only_(environment_only)
{
  if (horizontal_samples_ < 4 || vertical_samples_ < 1 || range_min_ < 0.0 ||
    range_max_ <= range_min_ || vertical_max_ < vertical_min_)
  {
    throw std::invalid_argument("invalid lidar sampling parameters");
  }
  char error[1024]{};
  model_.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
  if (!model_) {
    throw std::runtime_error("cannot load MuJoCo model: " + std::string(error));
  }
  data_.reset(mj_makeData(model_.get()));
  if (!data_) {
    throw std::runtime_error("cannot allocate MuJoCo data");
  }
  base_body_id_ = mj_name2id(model_.get(), mjOBJ_BODY, "base_link");
  if (base_body_id_ < 0 || model_->nq < 19) {
    throw std::runtime_error("official Go2 freejoint model contract is missing");
  }
  if (environment_only_) {
    ray_geom_groups_[0] = 1;
    for (int geom_id = 0; geom_id < model_->ngeom; ++geom_id) {
      model_->geom_group[geom_id] = is_robot_body(model_->geom_bodyid[geom_id]) ? 1 : 0;
    }
  }
  for (std::size_t index = 0; index < kMotorJointNames.size(); ++index) {
    const int joint_id = mj_name2id(model_.get(), mjOBJ_JOINT, kMotorJointNames[index]);
    if (joint_id < 0) {
      throw std::runtime_error("missing Go2 joint: " + std::string(kMotorJointNames[index]));
    }
    motor_qpos_addresses_[index] = model_->jnt_qposadr[joint_id];
  }
  build_sensor_directions();
  build_planar_ray_indices();
}

void LidarModel::build_sensor_directions()
{
  sensor_directions_.reserve(horizontal_samples_ * vertical_samples_ * 3);
  for (int vertical = 0; vertical < vertical_samples_; ++vertical) {
    const double ratio = vertical_samples_ == 1 ? 0.5 :
      static_cast<double>(vertical) / static_cast<double>(vertical_samples_ - 1);
    const double elevation = vertical_min_ + ratio * (vertical_max_ - vertical_min_);
    for (int horizontal = 0; horizontal < horizontal_samples_; ++horizontal) {
      const double azimuth = -kPi + 2.0 * kPi * horizontal / horizontal_samples_;
      sensor_directions_.push_back(std::cos(elevation) * std::cos(azimuth));
      sensor_directions_.push_back(std::cos(elevation) * std::sin(azimuth));
      sensor_directions_.push_back(std::sin(elevation));
    }
  }
}

void LidarModel::build_planar_ray_indices()
{
  const std::array<double, 4> mount_quaternion{
    std::cos(kLidarPitch / 2.0), 0.0, std::sin(kLidarPitch / 2.0), 0.0};
  std::vector<std::array<double, 3>> base_directions;
  base_directions.reserve(horizontal_samples_ * vertical_samples_);
  for (std::size_t ray = 0; ray < sensor_directions_.size() / 3; ++ray) {
    base_directions.push_back(rotate(
      mount_quaternion,
      {sensor_directions_[ray * 3], sensor_directions_[ray * 3 + 1],
        sensor_directions_[ray * 3 + 2]}));
  }
  planar_ray_indices_ = select_planar_ray_indices(base_directions, horizontal_samples_);
}

const std::vector<int> & LidarModel::planar_ray_indices() const
{
  return planar_ray_indices_;
}

void LidarModel::update_state(
  const std::array<double, 3> & imu_position,
  const std::array<double, 4> & base_quaternion_wxyz,
  const std::array<double, 12> & motor_positions)
{
  const auto quaternion = normalized(base_quaternion_wxyz);
  const auto rotated_imu_offset = rotate(quaternion, kImuOffset);
  for (int axis = 0; axis < 3; ++axis) {
    data_->qpos[axis] = imu_position[axis] - rotated_imu_offset[axis];
    data_->qpos[axis + 3] = quaternion[axis];
  }
  data_->qpos[6] = quaternion[3];
  for (std::size_t index = 0; index < motor_positions.size(); ++index) {
    data_->qpos[motor_qpos_addresses_[index]] = motor_positions[index];
  }
  mj_forward(model_.get(), data_.get());
}

bool LidarModel::is_robot_body(int body_id) const
{
  for (int current = body_id; current > 0; current = model_->body_parentid[current]) {
    if (current == base_body_id_) {
      return true;
    }
  }
  return false;
}

bool LidarModel::is_world_floor_geom(int geom_id) const
{
  return model_->geom_bodyid[geom_id] == 0 && model_->geom_type[geom_id] == mjGEOM_PLANE;
}

std::vector<RayHit> LidarModel::cast() const
{
  const std::array<double, 4> base_quaternion{
    data_->qpos[3], data_->qpos[4], data_->qpos[5], data_->qpos[6]};
  const auto lidar_offset = rotate(base_quaternion, kLidarTranslation);
  const mjtNum origin[3]{
    data_->qpos[0] + lidar_offset[0], data_->qpos[1] + lidar_offset[1],
    data_->qpos[2] + lidar_offset[2]};
  const mjtNum mount_quaternion[4]{std::cos(kLidarPitch / 2.0), 0.0, std::sin(kLidarPitch / 2.0), 0.0};
  const mjtNum base_quat[4]{base_quaternion[0], base_quaternion[1], base_quaternion[2], base_quaternion[3]};
  mjtNum world_quaternion[4]{};
  mjtNum world_rotation[9]{};
  mju_mulQuat(world_quaternion, base_quat, mount_quaternion);
  mju_quat2Mat(world_rotation, world_quaternion);

  const int ray_count = horizontal_samples_ * vertical_samples_;
  std::vector<mjtNum> world_directions(sensor_directions_.size());
  for (int ray = 0; ray < ray_count; ++ray) {
    mju_mulMatVec3(
      &world_directions[ray * 3], world_rotation, &sensor_directions_[ray * 3]);
  }
  std::vector<int> geom_ids(ray_count, -1);
  std::vector<mjtNum> distances(ray_count, -1.0);
  const mjtByte * geom_groups = environment_only_ ? ray_geom_groups_.data() : nullptr;
  mj_multiRay(
    model_.get(), data_.get(), origin, world_directions.data(), geom_groups, 1,
    base_body_id_, geom_ids.data(), distances.data(), ray_count, range_max_);

  std::vector<RayHit> hits;
  hits.reserve(ray_count);
  for (int ray = 0; ray < ray_count; ++ray) {
    if (geom_ids[ray] < 0 || distances[ray] < range_min_ || distances[ray] > range_max_) {
      continue;
    }
    const int body_id = model_->geom_bodyid[geom_ids[ray]];
    hits.push_back(RayHit{
      static_cast<float>(sensor_directions_[ray * 3] * distances[ray]),
      static_cast<float>(sensor_directions_[ray * 3 + 1] * distances[ray]),
      static_cast<float>(sensor_directions_[ray * 3 + 2] * distances[ray]),
      geom_ids[ray], body_id, is_robot_body(body_id),
      is_world_floor_geom(geom_ids[ray]), ray});
  }
  return hits;
}

}
