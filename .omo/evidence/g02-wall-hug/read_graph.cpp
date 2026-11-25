// Read saved Karto graph factors. Pose1/Pose2 and the relative measurement are
// fixed when LinkInfo is created; later graph optimization does not alter them.
// No ROS, optimizer, controller, or live simulator is started.
#include "karto_sdk/Mapper.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "read_graph INPUT.posegraph OUTPUT.jsonl\n";
    return 2;
  }
  karto::Mapper mapper;
  mapper.LoadFromFile(argv[1]);
  std::ofstream out(argv[2]);
  out << std::setprecision(17);
  for (const auto edge : mapper.GetGraph()->GetEdges()) {
    const auto from = edge->GetSource()->GetObject();
    const auto to = edge->GetTarget()->GetObject();
    const auto info = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    if (info == nullptr) {throw std::runtime_error("edge is not LinkInfo");}
    const auto a = info->GetPose1();
    const auto b = info->GetPose2();
    const auto d = info->GetPoseDifference();
    const auto covariance = info->GetCovariance();
    out << "{\"from_scan_id\":" << from->GetStateId()
        << ",\"to_scan_id\":" << to->GetStateId()
        << ",\"from_time\":" << from->GetTime() << ",\"to_time\":" << to->GetTime()
        << ",\"from_pose_at_link\":[" << a.GetX() << "," << a.GetY() << "," << a.GetHeading()
        << "],\"to_pose_at_link\":[" << b.GetX() << "," << b.GetY() << "," << b.GetHeading()
        << "],\"relative_pose\":[" << d.GetX() << "," << d.GetY() << "," << d.GetHeading()
        << "],\"covariance\":[";
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (i != 0 || j != 0) {out << ",";}
        out << covariance(i, j);
      }
    }
    out << "]}\n";
  }
  std::cout << "GRAPH_FACTORS_READ " << mapper.GetGraph()->GetEdges().size() << "\n";
}
