// Independent 10cm placement: wall near face y=.95; robot far face y=.85.
#include "clearance_2d.h"
#include <cassert>
#include <cstdio>
#include <cstring>
int main() {
  const char* xml="<mujoco><worldbody><geom type='box' pos='0 1 0' size='1 .05 1'/>"
    "<body pos='0 .8 .3'><freejoint/><geom type='box' size='.1 .05 .1'/></body>"
    "</worldbody></mujoco>";
  mjVFS vfs; mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs,"control.xml",xml,std::strlen(xml));
  char error[1024]{};
  mjModel* m=mj_loadXML("control.xml",&vfs,error,sizeof(error));
  if (!m) {std::fprintf(stderr,"%s\n",error);return 1;}
  mjData* d=mj_makeData(m);mj_forward(m,d);
  const double measured=clearance2d(m,d,0,1);
  assert(std::abs(measured-.10)<1e-9);
  std::printf("ANALYTIC_CLEARANCE_CONTROL expected=0.100000000 measured=%.9f\n",measured);
  mj_deleteData(d);mj_deleteModel(m);mj_deleteVFS(&vfs);
}
