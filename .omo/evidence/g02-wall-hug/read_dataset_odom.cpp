// Read exact serialized odometry from the real baseline dataset, without SLAM.
#include "karto_sdk/Mapper.h"
#include <fstream>
#include <iomanip>
#include <stdexcept>
int main(int argc,char ** argv)
{
  if (argc!=4) {throw std::runtime_error("read_dataset_odom INPUT.posegraph INPUT.data OUTPUT.csv");}
  karto::Mapper mapper;
  mapper.LoadFromFile(argv[1]);
  mapper.Reset();
  karto::Dataset dataset;
  dataset.LoadFromFile(argv[2]);
  for (const auto object:dataset.GetLasers()) {
    auto sensor=dynamic_cast<karto::Sensor *>(object);
    if (!sensor) {throw std::runtime_error("dataset object is not a sensor");}
    karto::SensorManager::GetInstance()->RegisterSensor(sensor,true);
  }
  std::ofstream out(argv[3]);
  out << std::setprecision(17) << "scan_id,time,x,y,yaw\n";
  for (const auto pair:dataset.GetData()) {
    const auto scan=dynamic_cast<karto::LocalizedRangeScan *>(pair.second);
    if (!scan) {throw std::runtime_error("dataset object is not a range scan");}
    const auto pose=scan->GetOdometricPose();
    out << scan->GetStateId() << "," << scan->GetTime() << "," << pose.GetX()
        << "," << pose.GetY() << "," << pose.GetHeading() << "\n";
  }
}
