// Passive LD_PRELOAD instrumentation of the actual GUI's mj_step. The original
// step runs exactly once with unchanged inputs. Record freejoint truth and all
// non-foot-floor contacts at every completed physics step. Every tenth step
// also measures analytic 2D collision-envelope clearance (not a driver input).
// This library is enabled only for the owned GUI via G02_PHYSICS_LOG.
#include <mujoco/mujoco.h>
#include <dlfcn.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include "clearance_2d.h"

extern "C" void mj_step(const mjModel* m, mjData* d)
{
  using Step = void (*)(const mjModel*, mjData*);
  static auto real = reinterpret_cast<Step>(dlsym(RTLD_NEXT, "mj_step"));
  real(m, d);
  static FILE* truth = nullptr;
  static FILE* contacts = nullptr;
  static long count = 0;
  if (!truth) {
    const char* root = std::getenv("G02_PHYSICS_LOG");
    if (!root) { return; }
    truth = std::fopen((std::string(root)+"-truth.csv").c_str(), "w");
    contacts = std::fopen((std::string(root)+"-contacts.csv").c_str(), "w");
    if (!truth || !contacts) { std::abort(); }
    std::fprintf(truth, "time,x,y,z,yaw,clearance,closest_world,closest_robot\n");
    std::fprintf(contacts, "time,geom1,geom2,dist,body,geom_id\n");
    std::fprintf(stderr,"G02_CLEARANCE_METRIC analytic_2d projected_collision_extents readonly\n");
    FILE* geometry=std::fopen((std::string(root)+"-geoms.csv").c_str(),"w");
    std::fprintf(geometry,"id,name,body,type,contype,affinity\n");
    for (int i=0;i<m->ngeom;++i)
      std::fprintf(geometry,"%d,%s,%d,%d,%d,%d\n",i,
                   mj_id2name(m,mjOBJ_GEOM,i)?mj_id2name(m,mjOBJ_GEOM,i):"",
                   m->geom_bodyid[i],m->geom_type[i],m->geom_contype[i],m->geom_conaffinity[i]);
    std::fclose(geometry);
  }
  for (int k=0; k<d->ncon; ++k) {
    const auto& c=d->contact[k];
    int a=c.geom[0],b=c.geom[1];
    if (a<0 || b<0 || (m->geom_bodyid[a]==0)==(m->geom_bodyid[b]==0)) { continue; }
    if (m->geom_bodyid[a]!=0) { std::swap(a,b); }
    const char* name=mj_id2name(m,mjOBJ_GEOM,b);
    const bool foot=name && (std::string(name)=="FL" || std::string(name)=="FR" ||
                            std::string(name)=="RL" || std::string(name)=="RR");
    if (!(foot && m->geom_type[a]==mjGEOM_PLANE) && c.dist<=0) {
      const char* world=mj_id2name(m,mjOBJ_GEOM,a);
      const char* body=mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[b]);
      std::fprintf(contacts,"%.6f,%s,%s,%.9f,%s,%d\n",d->time,world?world:"unnamed",
                   name?name:"unnamed",c.dist,body?body:"unnamed",b);
      std::fflush(contacts);
    }
  }
  double clearance=std::numeric_limits<double>::quiet_NaN();
  int closest_world=-1,closest_robot=-1;
  if ((count++ % 10)==0) {
    clearance=.75;
    for (int a=0;a<m->ngeom;++a) {
      if (m->geom_bodyid[a]!=0 || m->geom_type[a]==mjGEOM_PLANE) {continue;}
      for (int b=0;b<m->ngeom;++b) {
        if (m->geom_bodyid[b]==0 || !m->geom_contype[b]) {continue;}
        const double dx=d->geom_xpos[3*a]-d->geom_xpos[3*b];
        const double dy=d->geom_xpos[3*a+1]-d->geom_xpos[3*b+1];
        if (std::hypot(dx,dy)>m->geom_rbound[a]+m->geom_rbound[b]+clearance) {continue;}
        const double value=clearance2d(m,d,a,b);
        if (value<clearance) {
          clearance=value;
          closest_world=a;
          closest_robot=b;
        }
      }
    }
  }
  const double* q=d->qpos;
  const double yaw=std::atan2(2*(q[3]*q[6]+q[4]*q[5]),1-2*(q[5]*q[5]+q[6]*q[6]));
  std::fprintf(truth,"%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d\n",
               d->time,q[0],q[1],q[2],yaw,clearance,closest_world,closest_robot);
  if (count%100==0) {std::fflush(truth);}
}

// Tracking affects rendering only. MuJoCo updates lookat from this body's COM
// each frame, keeping the walking robot centered without writing physics data.
extern "C" void mjv_updateCamera(const mjModel* m,const mjData* d,
                                 mjvCamera* camera,mjvScene* scene)
{
  using Update=void (*)(const mjModel*,const mjData*,mjvCamera*,mjvScene*);
  static auto real=reinterpret_cast<Update>(dlsym(RTLD_NEXT,"mjv_updateCamera"));
  static bool configured=false;
  if (!configured) {
    camera->type=mjCAMERA_TRACKING;
    camera->trackbodyid=mj_name2id(m,mjOBJ_BODY,"base_link");
    camera->distance=6.5;
    camera->elevation=-70;
    camera->azimuth=90;
    configured=true;
    std::fprintf(stderr,"G02_TRACK_CAMERA body=%d type=%d distance=%.2f\n",
                 camera->trackbodyid,camera->type,camera->distance);
  }
  real(m,d,camera,scene);
}
