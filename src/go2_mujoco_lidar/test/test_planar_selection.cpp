// Hand-computed azimuth/elevation bins and hit flags are the independent oracle.
// GoogleTest checks remain active under Release/NDEBUG.
#include "go2_mujoco_lidar/lidar_model.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <vector>
#include <gtest/gtest.h>

namespace
{
std::array<double, 3> direction(double azimuth, double elevation)
{
  return {
    std::cos(elevation) * std::cos(azimuth),
    std::cos(elevation) * std::sin(azimuth),
    std::sin(elevation)};
}
}

TEST(PlanarSelection, ChoosesClosestElevationAndFirstTie)
{
  constexpr double pi = 3.14159265358979323846;
  const std::vector<std::array<double, 3>> directions{
    direction(-pi, 0.30), direction(-pi / 2.0, 0.20),
    direction(0.0, 0.40), direction(pi / 2.0, -0.30),
#ifdef W07_BROKEN_INPUT
    direction(-pi, -0.50), direction(-pi / 2.0, -0.20),
#else
    direction(-pi, -0.10), direction(-pi / 2.0, -0.20),
#endif
    direction(0.0, 0.05), direction(pi / 2.0, 0.01)};
  const auto original_directions = directions;

  const auto selected = go2_mujoco_lidar::select_planar_ray_indices(directions, 4);

  // Four quadrants: 0.10 < 0.30, equal 0.20 chooses the first,
  // 0.05 < 0.40, and 0.01 < 0.30.
  EXPECT_EQ(selected, (std::vector<int>{4, 1, 6, 7}));
  EXPECT_EQ(directions, original_directions);
}

TEST(PlanarSelection, ExcludesSelfFloorAndUnselectedHits)
{
  const std::vector<int> selected{4, 1, 6, 7};
  const std::vector<go2_mujoco_lidar::RayHit> hits{
    {1.0F, 0.0F, 0.0F, 10, 20, false, false, 4},
    {1.0F, 0.0F, 0.0F, 11, 21, true, false, 1},
    {1.0F, 0.0F, 0.0F, 12, 22, false, false, 7},
    {1.0F, 0.0F, 0.0F, 13, 0, false, true, 6},
    {1.0F, 0.0F, 0.0F, 14, 24, false, false, 0}};

  const auto planar_hits = go2_mujoco_lidar::select_planar_hits(hits, selected);

  ASSERT_EQ(planar_hits.size(), 2u);
  EXPECT_EQ(planar_hits[0].ray_index, 4);
  EXPECT_EQ(planar_hits[1].ray_index, 7);
}

TEST(PlanarSelection, EmptyBinsAndInvalidSampling)
{
  EXPECT_EQ(go2_mujoco_lidar::select_planar_ray_indices({{1, 0, 0}}, 4),
    (std::vector<int>{-1, -1, 0, -1}));
  EXPECT_THROW(go2_mujoco_lidar::select_planar_ray_indices({}, 0), std::invalid_argument);
  EXPECT_TRUE(go2_mujoco_lidar::select_planar_hits({}, {-1, -1}).empty());
}

TEST(RaycastGeometry, AnalyticWallsSelfAndFloor)
{
  // The +/-Y sensor rays are unchanged by the canonical pitch rotation.
  // Boxes centred at y=+/-2 with half-width 0.1 have faces at +/-1.9 m.
  const auto path = std::filesystem::temp_directory_path() /
    ("w07-geometry-" + std::to_string(getpid()) + ".xml");
  std::ofstream xml(path);
  xml << R"(<mujoco><worldbody>
    <geom type="plane" size="10 10 0.1"/>
    <geom type="box" pos="0 2 1" size="10 0.1 10"/>
    <geom type="box" pos="0 -2 1" size="10 0.1 10"/>
    <body name="base_link"><freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
      <site name="imu" pos="0 0 0"/>
  )";
  for (const auto * leg : {"FR", "FL", "RR", "RL"}) {
    for (const auto * joint : {"hip", "thigh", "calf"}) {
      xml << "<body><joint name=\"" << leg << "_" << joint
          << "_joint\"/><geom type=\"sphere\" pos=\"0 0 -5\" size=\"0.01\"/></body>";
    }
  }
  xml << R"(<body name="self"><geom type="box" pos="0 0.9 0"
    size="1 0.1 1"/></body></body></worldbody></mujoco>)";
  xml.close();
  go2_mujoco_lidar::BaseState base{{0, 0, 1}, {1, 0, 0, 0}, {}, {}};
  go2_mujoco_lidar::LidarModel raw(path.string(), 4, 1, 0, 0, 0, 10, false);
  raw.update_state(base, {});
  const auto raw_hits = raw.cast();
  EXPECT_TRUE(std::any_of(raw_hits.begin(), raw_hits.end(),
    [](const auto & hit) {return hit.robot_self;}));
  EXPECT_TRUE(std::any_of(raw_hits.begin(), raw_hits.end(),
    [](const auto & hit) {return hit.world_floor;}));
  go2_mujoco_lidar::LidarModel environment(path.string(), 4, 1, 0, 0, 0, 10, true);
  environment.update_state(base, {});
  int wall_hits = 0;
  for (const auto & hit : environment.cast()) {
    EXPECT_FALSE(hit.robot_self);
    if (hit.ray_index == 1 || hit.ray_index == 3) {
      ++wall_hits;
      EXPECT_NEAR(std::abs(hit.y), 1.9, 1e-6);
      EXPECT_NEAR(hit.x, 0, 1e-6);
      EXPECT_NEAR(hit.z, 0, 1e-6);
      EXPECT_FALSE(hit.world_floor);
    }
  }
  EXPECT_EQ(wall_hits, 2);
  std::filesystem::remove(path);
}
