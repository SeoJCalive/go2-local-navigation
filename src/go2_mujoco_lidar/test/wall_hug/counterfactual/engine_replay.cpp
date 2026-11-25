// Offline Karto replay: serialized raw ranges + original odometry -> matching,
// new loop decisions, Ceres optimization, final graph and occupancy grid.
// Derived from the G02 evidence replay_dataset.cpp. No recorded corrected pose,
// candidate prior, edge decision or ground truth is fed into the engine.
// Usage: engine_replay DATA YAML OUTPUT_PREFIX X K C MAX_INPUTS
// X=0 disables linked-node odometry-distance gate; K<0 disables correction gate.
// OUTPUT_PREFIX must have an existing parent. See ../README.md for input pins.
#include "karto_sdk/Mapper.h"
#include "solvers/ceres_solver.hpp"
#include "slam_toolbox/slam_mapper.hpp"
#include <tbb/global_control.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

extern "C" int go2ReplayPatchVersion();

namespace
{
double distanceLimit, fraction, constant, arc = 0., retainedArc = 0.;
std::ofstream decisions;
}

extern "C" bool go2ReplayGate(karto::LocalizedRangeScan * anchor,
  karto::LocalizedRangeScan * query, double correction)
{
  const double separation = std::sqrt(anchor->GetOdometricPose().GetPosition().
    SquaredDistance(query->GetOdometricPose().GetPosition()));
  const double distance = arc - retainedArc;
  const bool accept = (distanceLimit <= 0. || separation <= distanceLimit) &&
    (fraction < 0. || correction <= fraction * distance + constant);
  decisions << "{\"from_scan_id\":" << anchor->GetStateId()
    << ",\"to_scan_id\":" << query->GetStateId() << ",\"time\":" << query->GetTime()
    << ",\"separation_m\":" << separation << ",\"correction_m\":" << correction
    << ",\"distance_since_retained_m\":" << distance << ",\"accept\":"
    << (accept ? "true" : "false") << "}\n";
  return accept;
}

extern "C" void go2ReplayRetained()
{
  // Only an actually inserted new loop resets D; callbacks and rejected loops do not.
  retainedArc = arc;
}

static void pose(std::ostream & out, const karto::Pose2 & value)
{
  out << "[" << value.GetX() << "," << value.GetY() << "," << value.GetHeading() << "]";
}

int main(int argc, char ** argv)
{
  if (go2ReplayPatchVersion() != 1) {
    throw std::runtime_error("repo-only Karto replay patch is not loaded");
  }
  if (argc != 8) {
    throw std::runtime_error("engine_replay DATA YAML OUTPUT_PREFIX X K C MAX_INPUTS");
  }
  const std::string output = argv[3];
  distanceLimit = std::stod(argv[4]);
  fraction = std::stod(argv[5]);
  constant = std::stod(argv[6]);
  // argv[7] is a maximum input count for bounded regression probes; 0 means all.
  const auto maximum = std::stoul(argv[7]);
  decisions.open(output + "-decisions.jsonl");
  decisions << std::setprecision(17);
  setenv("G02_EDGE_EVIDENCE", (output + "-edges.jsonl").c_str(), 1);
  unsetenv("G02_SCAN_EVIDENCE");
  unsetenv("G02_CANDIDATE_EVIDENCE");
  // Match and solve with one worker to avoid parallel floating point reductions.
  tbb::global_control workers(tbb::global_control::max_allowed_parallelism, 1);
  int ros_argc = 1;
  rclcpp::init(ros_argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", argv[2]});
  options.parameter_overrides({rclcpp::Parameter("debug_logging", false),
    rclcpp::Parameter("use_sim_time", true)});
  auto node = rclcpp::Node::make_shared("slam_toolbox", options);
  node->declare_parameter("debug_logging", false);
  mapper_utils::SMapper configured;
  configured.configure(node);
  auto & mapper = *configured.getMapper();
  if (mapper.getParamLoopMatchMaximumVarianceCoarse() != 3.) {
    throw std::runtime_error("counterfactual contract requires unchanged sigma 3.0");
  }
  solver_plugins::CeresSolver solver;
  solver.Configure(node);
  mapper.SetScanSolver(&solver);
  karto::Dataset dataset;
  dataset.LoadFromFile(argv[1]);
  for (auto object : dataset.GetLasers()) {
    auto sensor = dynamic_cast<karto::Sensor *>(object);
    if (!sensor) {throw std::runtime_error("invalid dataset sensor");}
    karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);
  }
  std::ofstream online(output + "-online.jsonl"), inputs(output + "-inputs.jsonl");
  online << std::setprecision(17);
  inputs << std::setprecision(17);
  karto::Pose2 previous;
  size_t count = 0, filtered = 0;
  for (auto pair : dataset.GetData()) {
    auto source = dynamic_cast<karto::LocalizedRangeScan *>(pair.second);
    if (!source) {throw std::runtime_error("dataset contains a non-scan object");}
    if (maximum && count >= maximum) {break;}
    const auto odom = source->GetOdometricPose();
    if (count) {arc += std::sqrt(previous.GetPosition().SquaredDistance(odom.GetPosition()));}
    previous = odom;
    auto scan = new karto::LocalizedRangeScan(
      source->GetSensorName(), source->GetRangeReadingsVector());
    scan->SetTime(source->GetTime());
    scan->SetOdometricPose(odom);
    scan->SetCorrectedPose(odom);
    inputs << "{\"source_id\":" << source->GetStateId() << ",\"time\":"
      << source->GetTime() << ",\"odom\":";
    pose(inputs, odom);
    inputs << ",\"arc_m\":" << arc << ",\"laser_offset\":";
    pose(inputs, source->GetLaserRangeFinder()->GetOffsetPose());
    inputs << "}\n";
    const bool accepted = mapper.Process(scan);
    online << "{\"source_id\":" << source->GetStateId() << ",\"time\":"
      << source->GetTime() << ",\"accepted\":" << (accepted ? "true" : "false");
    if (accepted) {
      online << ",\"id\":" << scan->GetStateId() << ",\"pose\":";
      pose(online, scan->GetCorrectedPose());
    } else {
      delete scan;
      ++filtered;
    }
    online << "}\n";
    ++count;
    if (count % 100 == 0) {std::cout << "REPLAY_PROGRESS " << count << std::endl;}
  }
  mapper.SaveToFile(output + ".posegraph");
  std::ofstream vertices(output + "-vertices.jsonl"), factors(output + "-factors.jsonl");
  vertices << std::setprecision(17);
  factors << std::setprecision(17);
  for (auto scan : mapper.GetAllProcessedScans()) {
    vertices << "{\"id\":" << scan->GetStateId() << ",\"time\":" << scan->GetTime()
      << ",\"pose\":";
    pose(vertices, scan->GetCorrectedPose());
    vertices << ",\"odom\":";
    pose(vertices, scan->GetOdometricPose());
    vertices << "}\n";
  }
  for (auto edge : mapper.GetGraph()->GetEdges()) {
    auto from = edge->GetSource()->GetObject(), to = edge->GetTarget()->GetObject();
    auto info = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    factors << "{\"from_scan_id\":" << from->GetStateId() << ",\"to_scan_id\":"
      << to->GetStateId() << ",\"from_time\":" << from->GetTime() << ",\"to_time\":"
      << to->GetTime() << ",\"relative_pose\":";
    pose(factors, info->GetPoseDifference());
    factors << "}\n";
  }
  auto grid = configured.getOccupancyGrid(.05);
  if (!grid) {throw std::runtime_error("replay produced no occupancy grid");}
  const auto offset = grid->GetCoordinateConverter()->GetOffset();
  std::ofstream pgm(output + ".pgm", std::ios::binary);
  pgm << "P5\n" << grid->GetWidth() << " " << grid->GetHeight() << "\n255\n";
  for (int y = grid->GetHeight() - 1; y >= 0; --y) {
    for (int x = 0; x < grid->GetWidth(); ++x) {
      auto cell = *grid->GetDataPointer(karto::Vector2<kt_int32s>(x, y));
      const unsigned char value = cell == karto::GridStates_Occupied ? 0 :
        cell == karto::GridStates_Free ? 254 : 205;
      pgm.write(reinterpret_cast<const char *>(&value), 1);
    }
  }
  std::ofstream yaml(output + ".yaml");
  yaml << std::setprecision(17) << "image: "
    << output.substr(output.find_last_of('/') + 1) << ".pgm\nresolution: 0.05\norigin: ["
    << offset.GetX() << ", " << offset.GetY()
    << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  delete grid;
  mapper.SetScanSolver(nullptr);
  rclcpp::shutdown();
  std::cout << "REPLAY_FINISHED inputs=" << count << " movement_filtered=" << filtered
    << " sigma=3 workers=1" << std::endl;
}
