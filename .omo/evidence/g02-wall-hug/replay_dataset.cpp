// Replay saved actual scans through the same Karto parameters and installed
// Ceres optimizer. No simulator, FIFO, motion, or ROS executor is started.
// Baseline native edge equality is required before using recovered chain sizes.
#include "karto_sdk/Mapper.h"
#include "solvers/ceres_solver.hpp"
#include "slam_toolbox/slam_mapper.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if (argc != 5) {
    throw std::runtime_error("replay_dataset BASE.data GUI_PARAMS.yaml OUTPUT.posegraph OUTPUT_MAPPING.jsonl");
  }
  int ros_argc = 1;
  rclcpp::init(ros_argc, argv);
  // The exact GUI YAML goes through ROS parameter loading and the same
  // SMapper::configure function as the GUI node, rather than copied setters.
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args","--params-file",argv[2]});
  options.parameter_overrides({
    rclcpp::Parameter("debug_logging",true),
    rclcpp::Parameter("use_sim_time",true)
  });
  auto node = rclcpp::Node::make_shared("slam_toolbox",options);
  node->declare_parameter("debug_logging", false);
  mapper_utils::SMapper configured;
  configured.configure(node);
  karto::Mapper & mapper=*configured.getMapper();
  solver_plugins::CeresSolver solver;
  solver.Configure(node);
  mapper.SetScanSolver(&solver);
  karto::Dataset dataset;
  dataset.LoadFromFile(argv[1]);
  for (auto object : dataset.GetLasers()) {
    auto sensor = dynamic_cast<karto::Sensor *>(object);
    if (sensor == nullptr) {throw std::runtime_error("invalid dataset sensor");}
    karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);
  }
  std::cout << "REPLAY_PARAMETERS dimension=" << mapper.getParamLoopSearchSpaceDimension()
            << " sigma=" << mapper.getParamLoopMatchMaximumVarianceCoarse()
            << " internal_variance=" << std::pow(mapper.getParamLoopMatchMaximumVarianceCoarse(),2)
            << " coarse=" << mapper.getParamLoopMatchMinimumResponseCoarse()
            << " fine=" << mapper.getParamLoopMatchMinimumResponseFine()
            << " chain=" << mapper.getParamLoopMatchMinimumChainSize()
            << " candidate_distance=" << mapper.getParamLoopSearchMaximumDistance()
            << std::endl;
  std::ofstream mapping(argv[4]);
  mapping << std::setprecision(17);
  size_t count = 0, rejected = 0;
  for (auto pair : dataset.GetData()) {
    auto source = dynamic_cast<karto::LocalizedRangeScan *>(pair.second);
    if (source == nullptr) {throw std::runtime_error("invalid saved range scan");}
    auto scan = new karto::LocalizedRangeScan(
      source->GetSensorName(), source->GetRangeReadingsVector());
    scan->SetTime(source->GetTime());
    scan->SetOdometricPose(source->GetOdometricPose());
    scan->SetCorrectedPose(source->GetOdometricPose());
    if (!mapper.Process(scan)) {
      mapping << "{\"source_id\":" << source->GetStateId()
              << ",\"time\":" << source->GetTime() << ",\"accepted\":false}\n";
      delete scan;
      ++rejected;
      continue;
    }
    mapping << "{\"source_id\":" << source->GetStateId()
            << ",\"replay_id\":" << scan->GetStateId() << ",\"time\":"
            << source->GetTime() << ",\"accepted\":true}\n";
    ++count;
    if (count % 100 == 0) {std::cout << "REPLAY_PROGRESS " << count << std::endl;}
  }
  mapper.SaveToFile(argv[3]);
  std::cout << "REPLAY_FINISHED accepted=" << count << " movement_filtered="
            << rejected << std::endl;
  mapper.SetScanSolver(nullptr);
  rclcpp::shutdown();
}
