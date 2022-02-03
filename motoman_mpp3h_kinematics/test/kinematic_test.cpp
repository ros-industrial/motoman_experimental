#include "motoman_mpp3h_kinematics/mpp3h_kinematics.h"
#include <gtest/gtest.h>

TEST(MPP3HKinematic, testForwardInverseKinematic) {

  Mpp3hKinematics mpp3hkinematics;
  boost::array<double,4> joints;
  boost::array<double,4> joints_after_kin;
  double tool_ang;
  tf::Vector3 point;

  joints[0] = 1;
  joints[1] = 0.7;
  joints[2] = 0.1;
  joints[3] = 0.2;

  mpp3hkinematics.forwardKinematics(joints, point, tool_ang);
  mpp3hkinematics.inverseKinematics(point, tool_ang, joints_after_kin);

  for (int i=0; i<joints.size(); ++i) {
    EXPECT_NEAR(joints[i], joints_after_kin[i], 1e-12);
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


