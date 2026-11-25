// Humble MapDisplay::reset clears its texture on a backwards /clock jump but
// retains the subscription. A one-shot transient-local map is then never read
// again. Recreate that subscription at reset so DDS replays the durable sample.
// No map/TF publisher is introduced; Map Server/SLAM retain their ownership.
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace go2_mujoco_lidar
{
class TransientMapDisplay : public rviz_default_plugins::displays::MapDisplay
{
public:
  void reset() override
  {
    MapDisplay::reset();
    if (isEnabled()) {
      unsubscribe();
      subscribe();
    }
  }
};
}

PLUGINLIB_EXPORT_CLASS(go2_mujoco_lidar::TransientMapDisplay, rviz_common::Display)
