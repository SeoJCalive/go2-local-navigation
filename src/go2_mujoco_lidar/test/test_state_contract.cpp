// Independent analytic geometry and DDS ordering checks for base state and epochs.
// Explicit failures keep all checks active under Release/NDEBUG.
#include "go2_mujoco_lidar/lidar_model.hpp"
#include "go2_mujoco_lidar/step_pairing.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using go2_mujoco_lidar::base_state_from_imu;
using go2_mujoco_lidar::StepPairing;

namespace
{
int checks = 0;
void check(bool condition)
{
  ++checks;
  if (!condition) {
    throw std::runtime_error("state contract check " + std::to_string(checks));
  }
}
void close(double actual, double expected)
{
  check(std::abs(actual - expected) < 1.0e-12);
}
auto sample(int value) {return std::make_shared<const int>(value);}
}

int main()
{
  try {
    // r=(1,2,3), base=(4,5,6). At yaw +90deg, Rr=(-2,1,3).
    const double half = std::sqrt(0.5);
    const auto positive = base_state_from_imu(
      {2, 6, 9}, {half, 0, 0, half}, {0, 2, 0}, {0, 0, 0}, {1, 2, 3});
    close(positive.position[0], 4); close(positive.position[1], 5);
    close(positive.position[2], 6);
    close(positive.linear_velocity[0], 2); close(positive.linear_velocity[1], 0);
    const auto negative = base_state_from_imu(
      {6, 4, 9}, {half, 0, 0, -half}, {0, -2, 0}, {0, 0, 0}, {1, 2, 3});
    close(negative.position[0], 4); close(negative.position[1], 5);
    close(negative.linear_velocity[0], 2); close(negative.linear_velocity[1], 0);
    // Pure yaw rate 2 about a stationary base: omega x r=(-4,2,0).
    // Rotate that by +90deg to world (-2,-4,0); base linear twist must be zero.
    const auto spinning = base_state_from_imu(
      {2, 6, 9}, {half * 2, 0, 0, half * 2},
      {-2, -4, 0}, {0, 0, 2}, {1, 2, 3});
    for (const auto value : spinning.linear_velocity) {close(value, 0);}
    close(spinning.angular_velocity[2], 2);
    const auto identity = base_state_from_imu(
      {5, 7, 9}, {1, 0, 0, 0}, {2, 3, 4}, {0, 0, 0}, {1, 2, 3});
    close(identity.position[0], 4); close(identity.position[1], 5);
    close(identity.position[2], 6);
    close(identity.linear_velocity[0], 2); close(identity.linear_velocity[1], 3);
    close(identity.linear_velocity[2], 4);
    // Roll +90deg exercises the vertical axis and full 3D lever-arm correction.
    const auto rolling = base_state_from_imu(
      {5, 2, 8}, {half, half, 0, 0}, {0, -4, -6}, {2, 0, 0}, {1, 2, 3});
    close(rolling.position[0], 4); close(rolling.position[1], 5);
    close(rolling.position[2], 6);
    for (const auto value : rolling.linear_velocity) {close(value, 0);}

    StepPairing<int, int> pairs;
    pairs.add_low(10, sample(10));
    pairs.add_sport(11, sample(11));
    check(!pairs.take());  // Adjacent steps never join.
    pairs.add_low(11, sample(111));
    auto pair = pairs.take();
    check(pair && *pair->low == 111 && *pair->sport == 11);
    check(!pairs.take());
    pairs.add_low(11, sample(0)); pairs.add_sport(11, sample(0));
    check(!pairs.take());  // Duplicate timestamp cannot regenerate output.
    pairs.add_sport(12, sample(12)); pairs.add_low(12, sample(12));
    check(pairs.take().has_value());  // Both callback orders work.
    check(pairs.add_sport(1, sample(1)));  // Reset begins on Sport.
    pairs.add_low(13, sample(13));  // Old low epoch is still arriving.
    check(!pairs.take());
    check(!pairs.add_low(1, sample(1)));  // Second regression acknowledges reset.
    check(pairs.take().has_value());
    check(!pairs.take());
    pairs.add_low(2, sample(2)); pairs.add_sport(2, sample(2));
    check(pairs.take().has_value());
    check(pairs.add_low(0, sample(0)));  // Reset may also begin on Low.
    check(!pairs.add_sport(0, sample(0)));
    check(pairs.take().has_value());
    for (int step = 1; step < 1000; ++step) {pairs.add_low(step, sample(step));}
    const auto bounded_size = pairs.pending_size();
    for (int step = 1000; step < 2000; ++step) {pairs.add_low(step, sample(step));}
    check(pairs.pending_size() == bounded_size && bounded_size < 1000);
    check(pairs.dropped() > 0);
    check(!pairs.take());  // Missing Sport produces no cached output.
    pairs.add_sport(1999, sample(1999));
    check(pairs.take().has_value());
    StepPairing<int, int> late_peer;
    late_peer.add_sport(100, sample(100));
    check(late_peer.add_sport(1, sample(1)));
    late_peer.add_low(1, sample(1));
    check(late_peer.take().has_value());  // A never-seen peer has no old epoch to drain.
    bool rejected = false;
    try {
      base_state_from_imu({0, 0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0, 0, 0},
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0});
    } catch (const std::runtime_error &) {rejected = true;}
    check(rejected);
    std::cout << "PASS state_contract checks=" << checks << '\n';
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
