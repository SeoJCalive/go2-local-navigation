// Diagnostic replay only: restore the actually recorded query hint before the
// original TryCloseLoop call. This removes accumulated optimizer roundoff from
// the query. No GT, fabricated scan, or forced acceptance decision is supplied.
// This library is never loaded by a simulator/GUI or a product launch.
#include "karto_sdk/Mapper.h"
#include <cmath>
#include <cstdlib>
#include <dlfcn.h>
#include <fstream>
#include <map>
#include <stdexcept>

kt_bool karto::MapperGraph::TryCloseLoop(
    karto::LocalizedRangeScan * scan, const karto::Name & name)
{
  struct Prior {double time; karto::Pose2 pose;};
  static const auto priors=[] {
    const char * path=std::getenv("G02_REPLAY_RECORDED_PRIORS");
    if (path==nullptr) {throw std::runtime_error("replay prior file required");}
    std::ifstream input(path);
    std::map<int,Prior> result;
    int id; double stamp,x,y,yaw;
    while (input>>id>>stamp>>x>>y>>yaw) {
      if (!result.emplace(id,Prior{stamp,karto::Pose2(x,y,yaw)}).second) {
        throw std::runtime_error("duplicate recorded query ID");
      }
    }
    if (!input.eof() || result.empty()) {
      throw std::runtime_error("invalid recorded query priors");
    }
    return result;
  }();
  const auto prior=priors.at(scan->GetStateId());
  if (std::abs(prior.time-scan->GetTime())>1e-6) {
    throw std::runtime_error("recorded query stamp differs from replay");
  }
  scan->SetSensorPose(prior.pose);
  using Original=kt_bool (*)(karto::MapperGraph*,karto::LocalizedRangeScan*,
                            const karto::Name&);
  static auto original=reinterpret_cast<Original>(dlsym(
      RTLD_NEXT,"_ZN5karto11MapperGraph12TryCloseLoopEPNS_18LocalizedRangeScanERKNS_4NameE"));
  if (original==nullptr) {throw std::runtime_error("original TryCloseLoop missing");}
  return original(this,scan,name);
}
