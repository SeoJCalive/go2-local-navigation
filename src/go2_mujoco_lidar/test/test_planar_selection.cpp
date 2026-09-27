#include "go2_mujoco_lidar/lidar_model.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

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

int main()
{
  constexpr double pi = 3.14159265358979323846;
  const std::vector<std::array<double, 3>> directions{
    direction(-pi, 0.30), direction(-pi / 2.0, 0.20),
    direction(0.0, 0.40), direction(pi / 2.0, -0.30),
    direction(-pi, -0.10), direction(-pi / 2.0, -0.20),
    direction(0.0, 0.05), direction(pi / 2.0, 0.01)};
  const auto original_directions = directions;

  const auto selected = go2_mujoco_lidar::select_planar_ray_indices(directions, 4);

  assert((selected == std::vector<int>{4, 1, 6, 7}));
  assert(directions == original_directions);

  const std::vector<go2_mujoco_lidar::RayHit> hits{
    {1.0F, 0.0F, 0.0F, 10, 20, false, false, 4},
    {1.0F, 0.0F, 0.0F, 11, 21, true, false, 1},
    {1.0F, 0.0F, 0.0F, 12, 22, false, false, 7},
    {1.0F, 0.0F, 0.0F, 13, 0, false, true, 6},
    {1.0F, 0.0F, 0.0F, 14, 24, false, false, 0}};

  const auto planar_hits = go2_mujoco_lidar::select_planar_hits(hits, selected);

  assert(planar_hits.size() == 2);
  assert(planar_hits[0].ray_index == 4);
  assert(planar_hits[1].ray_index == 7);
  return 0;
}
