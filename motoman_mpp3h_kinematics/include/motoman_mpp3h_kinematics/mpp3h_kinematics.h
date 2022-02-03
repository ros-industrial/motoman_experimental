#ifndef MPP3H_KINEMATICS_H
#define MPP3H_KINEMATICS_H

#include <boost/array.hpp>
#include <cmath> 
#include <tf/tf.h>

class Mpp3hKinematics{
  public:
    Mpp3hKinematics():
    // MPP3H dimensions
    dist_armfixt_to_tool_(0.070),
    dist_lower_arm_links_(0.050),
    dist_motor_to_center_(0.160),
    lower_arm_len_(0.960),
    upper_arm_len_(0.360),
    geo_ang_(M_PI/3*2),  // 120 deg arrangement of the three arms
    shift_ang_(M_PI/2)   // MPP3H-COOS is shifted by 90 deg with respect to kinematic model
    {}

    // Joints: 
    //  The boost array of length 4 refers to the joint-names 
    //  [joint_s, joint_l, joint_u, joint_t] of MotoROS.
    //  The joints [joint_r, joint_b] are also published by MotoROS
    //  but are not used for the MPP3H delta robot. 


    // Calculate the joint angles for a given cartesian pose
    bool inverseKinematics(tf::Vector3 cartesian_pt,
                           double tool_ang,
                           boost::array<double,4> &joints) const;
    // Calculate the cartesian pose for given joint angles
    bool forwardKinematics(const boost::array<double,4> &joints,
                           tf::Vector3 &cartesian_pt,
                           double &tool_ang) const;
    // Calculate the transforms for the floating joints of the robot model
    bool getTransforms(const boost::array<double,4> &joints,
                       tf::Transform &tool, 
                       tf::Transform &arm_s_left, 
                       tf::Transform &arm_s_right, 
                       tf::Transform &arm_l_left, 
                       tf::Transform &arm_l_right, 
                       tf::Transform &arm_u_left, 
                       tf::Transform &arm_u_right) const;

  private:
    const double dist_armfixt_to_tool_, dist_lower_arm_links_, dist_motor_to_center_;
    const double lower_arm_len_, upper_arm_len_;
    const double geo_ang_, shift_ang_;

    // Helper function - inverse kinematic for single arm
    bool calcSingleAng(tf::Vector3 cartesian_pt,
                       double ang,
                       double &theta) const;
    // Helper function - calculate transform for single arm
    void calcSingleArmTransform(double ang,
                                double joint_value,
                                const tf::Transform &tool,                
                                tf::Transform &lower_arm_left, 
                                tf::Transform &lower_arm_right) const;
};
#endif 
