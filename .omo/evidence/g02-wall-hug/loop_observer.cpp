// Observe the installed Karto 2.6.10 loop algorithm without changing decisions.
// A same-version KARTO_DEBUG build supplies candidate BEST POSE output. This
// member-function interposition adds exact scan stamps and real listener events.
// Enabled only in the owned mapping launch, never in system installations.
#include <karto_sdk/Mapper.h>
#include <dlfcn.h>
#include <cstdio>
#include <algorithm>

namespace {
class Observer : public karto::MapperLoopClosureListener
{
public:
  int id;
  double time;
  void emit(const char* event,const std::string& info) {
    auto line=info;
    std::replace(line.begin(),line.end(),'\n',' ');
    std::fprintf(stderr,"G02_LOOP event=%s scan=%d time=%.9f info=%s\n",
                 event,id,time,line.c_str());
    std::fflush(stderr);
  }
  void LoopClosureCheck(const std::string& info) override {emit("check",info);}
  void BeginLoopClosure(const std::string& info) override {emit("begin",info);}
  void EndLoopClosure(const std::string& info) override {emit("accepted",info);}
};
}

kt_bool karto::MapperGraph::TryCloseLoop(
    karto::LocalizedRangeScan* scan,const karto::Name& name)
{
  using Original=kt_bool (*)(karto::MapperGraph*,karto::LocalizedRangeScan*,
                            const karto::Name&);
  static auto original=reinterpret_cast<Original>(dlsym(
      RTLD_NEXT,"_ZN5karto11MapperGraph12TryCloseLoopEPNS_18LocalizedRangeScanERKNS_4NameE"));
  Observer observer;
  observer.id=scan->GetStateId();
  observer.time=scan->GetTime();
  observer.emit("scan_begin","");
  m_pMapper->AddListener(&observer);
  const auto result=original(this,scan,name);
  m_pMapper->RemoveListener(&observer);
  observer.emit("scan_end",result?"accepted":"no_accepted_closure");
  return result;
}
