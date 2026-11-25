// Analytic horizontal collision envelopes. No CCD calls and no physics changes.
// Boxes use projected half-extents; capsules/cylinders use their exact support
// extent along each horizontal axis. Rotated robot envelopes are conservative
// rectangles in the wall's axes; near an open wall face its normal gap is exact.
#pragma once
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>

inline double extent2d(const mjModel* m,const mjData* d,int b,double ax,double ay) {
  const auto* r=d->geom_xmat+9*b;
  const auto* s=m->geom_size+3*b;
  const double x=ax*r[0]+ay*r[3],y=ax*r[1]+ay*r[4],z=ax*r[2]+ay*r[5];
  switch (m->geom_type[b]) {
    case mjGEOM_BOX:return std::abs(x)*s[0]+std::abs(y)*s[1]+std::abs(z)*s[2];
    case mjGEOM_SPHERE:return s[0];
    case mjGEOM_CAPSULE:return s[0]+std::abs(z)*s[1];
    case mjGEOM_CYLINDER:return s[0]*std::sqrt(std::max(0.,1-z*z))+std::abs(z)*s[1];
    case mjGEOM_ELLIPSOID:return std::sqrt(x*x*s[0]*s[0]+y*y*s[1]*s[1]+z*z*s[2]*s[2]);
    default:std::abort();
  }
}

inline double rectangleGap(double x,double y,double hx,double hy) {
  const double qx=std::abs(x)-hx,qy=std::abs(y)-hy;
  return std::hypot(std::max(qx,0.),std::max(qy,0.))+std::min(std::max(qx,qy),0.);
}

inline double clearance2d(const mjModel* m,const mjData* d,int world,int robot) {
  const double dx=d->geom_xpos[3*robot]-d->geom_xpos[3*world];
  const double dy=d->geom_xpos[3*robot+1]-d->geom_xpos[3*world+1];
  const auto* s=m->geom_size+3*world;
  if (m->geom_type[world]==mjGEOM_BOX) {
    const auto* r=d->geom_xmat+9*world;
    return rectangleGap(dx*r[0]+dy*r[3],dx*r[1]+dy*r[4],
        s[0]+extent2d(m,d,robot,r[0],r[3]),
        s[1]+extent2d(m,d,robot,r[1],r[4]));
  }
  if (m->geom_type[world]==mjGEOM_CYLINDER)
    return rectangleGap(dx,dy,extent2d(m,d,robot,1,0),extent2d(m,d,robot,0,1))-s[0];
  std::abort();
}
