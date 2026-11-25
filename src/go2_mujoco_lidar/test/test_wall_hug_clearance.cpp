// Purpose: independent analytic clearance controls, including body and leg shapes.
// Flow: build tiny MuJoCo geometry models, call support/clearance and compare hand calculations.
// Inputs: inline MJCF; output: GoogleTest failures. No stepping, DDS or robot motion.
// External: MuJoCo 3.3.6; see wall_hug/README.md. Corner envelopes are conservative.
#include "wall_hug/clearance_2d.h"
#include <gtest/gtest.h>
#include <cstring>
#include <string>

class Clearance : public ::testing::Test {
 protected:
  mjModel* m=nullptr; mjData* d=nullptr;
  void load(const std::string& robot, const std::string& world="type='box' pos='0 1 0' size='1 .05 1'") {
    auto xml="<mujoco><compiler angle='radian'/><worldbody><geom "+world+"/><body pos='0 .8 .3'><freejoint/><geom "+robot+"/></body></worldbody></mujoco>";
    mjVFS vfs; mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs,"control.xml",xml.c_str(),xml.size());
    char error[1024]{};
    m=mj_loadXML("control.xml",&vfs,error,sizeof(error));
    mj_deleteVFS(&vfs);
    ASSERT_NE(m,nullptr) << error;
    d=mj_makeData(m); mj_forward(m,d);
  }
  void TearDown() override {if(d) mj_deleteData(d); if(m) mj_deleteModel(m);}
};
TEST_F(Clearance, BodyTenCentimetresAndPenetration) {
  load("type='box' size='.1 .05 .1'"); ASSERT_NE(d,nullptr);
  EXPECT_NEAR(clearance2d(m,d,0,1),.1,1e-9);
  d->qpos[1]=.92; mj_forward(m,d);
  EXPECT_NEAR(clearance2d(m,d,0,1),-.02,1e-9);
}
TEST_F(Clearance, RotatedLegCapsule) {
  load("type='capsule' size='.02 .1' euler='1.5707963267948966 0 0'"); ASSERT_NE(d,nullptr);
  EXPECT_NEAR(extent2d(m,d,1,0,1),.12,1e-9);
  EXPECT_NEAR(clearance2d(m,d,0,1),.03,1e-9);
}
TEST_F(Clearance, UprightLegCylinder) {
  load("type='cylinder' size='.02 .1'"); ASSERT_NE(d,nullptr);
  EXPECT_NEAR(extent2d(m,d,1,0,1),.02,1e-9);
  EXPECT_NEAR(clearance2d(m,d,0,1),.13,1e-9);
}
TEST_F(Clearance, SphereAgainstCylinderWorld) {
  load("type='sphere' size='.05'", "type='cylinder' pos='0 1 0' size='.05 1'"); ASSERT_NE(d,nullptr);
  EXPECT_NEAR(clearance2d(m,d,0,1),.1,1e-9);
}
TEST_F(Clearance, EllipsoidSupport) {
  load("type='ellipsoid' size='.1 .05 .2'"); ASSERT_NE(d,nullptr);
  EXPECT_NEAR(extent2d(m,d,1,0,1),.05,1e-9);
  EXPECT_NEAR(extent2d(m,d,1,std::sqrt(.5),std::sqrt(.5)),std::sqrt(.00625),1e-9);
}
