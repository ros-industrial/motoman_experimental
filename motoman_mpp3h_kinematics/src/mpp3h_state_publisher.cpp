#include "motoman_mpp3h_kinematics/mpp3h_kinematics.h"
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class Mpp3hStatePublisher {

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster br_;
    std::string prefix_;
    std::string root_;
    std::vector<std::string> joint_names_;
    Mpp3hKinematics mpp3hkinematics_;

  public:
    Mpp3hStatePublisher(): nh_(), nh_priv_("~") {
      sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &Mpp3hStatePublisher::updateJointPosition_cb, this);
      nh_priv_.getParam("prefix",prefix_);
      root_ = prefix_+"base_link";     

      //  The boost array of length 4 refers to the joint-names
      //  The joints [joint_r, joint_b] are also published by MotoROS
      //  but are not used for the MPP3H delta robot.
      std::string joint_n [] = {prefix_+"joint_s", prefix_+"joint_l", prefix_+"joint_u", prefix_+"joint_t"};
      joint_names_.assign(joint_n, joint_n + sizeof(joint_n)/sizeof(joint_n[0]));
    }
    void updateJointPosition_cb(const sensor_msgs::JointState joints_msg);
    bool fillJointsByName(const sensor_msgs::JointState joints_msg,
                          boost::array<double,4> &joints);
};

bool Mpp3hStatePublisher::fillJointsByName(const sensor_msgs::JointState joints_msg,
                                           boost::array<double,4> &joints) {

    int found_joints = 0;
    for (int i=0; i<joints.size(); ++i) {
        for (int j=0; j<joints_msg.name.size(); ++j) {
            if (joints_msg.name[j] == joint_names_[i]) {
              joints[i] = joints_msg.position[j];
              ++found_joints;
            }
        }
    }
    if (found_joints==0){
      ROS_DEBUG("The published joint_states do not contain the joints of the MPP3H robot.");
      return false;
    }
    else if(found_joints<joints.size()) {
      ROS_ERROR("Only a subset of joints found.");
      return false;
    }
    return true;
}

void Mpp3hStatePublisher::updateJointPosition_cb(const sensor_msgs::JointState joints_msg) {

  boost::array<double,4> joints;
  tf::Transform tool, arm_s_left, arm_s_right, arm_l_left, arm_l_right, arm_u_left, arm_u_right;

  if(!fillJointsByName(joints_msg, joints)) {
      return;
  }

  if (!mpp3hkinematics_.getTransforms(joints, tool, arm_s_left, arm_s_right, arm_l_left, arm_l_right, arm_u_left, arm_u_right)) {
      ROS_ERROR("Could not calculate FK for given pose");
      return;
  }

  // Publish transformations
  br_.sendTransform(tf::StampedTransform(tool,        ros::Time::now(), root_, prefix_+"ee_upper_link"));
  br_.sendTransform(tf::StampedTransform(arm_s_left,  ros::Time::now(), root_, prefix_+"link_s_lower_left"));
  br_.sendTransform(tf::StampedTransform(arm_s_right, ros::Time::now(), root_, prefix_+"link_s_lower_right"));
  br_.sendTransform(tf::StampedTransform(arm_l_left,  ros::Time::now(), root_, prefix_+"link_l_lower_left"));
  br_.sendTransform(tf::StampedTransform(arm_l_right, ros::Time::now(), root_, prefix_+"link_l_lower_right"));
  br_.sendTransform(tf::StampedTransform(arm_u_left,  ros::Time::now(), root_, prefix_+"link_u_lower_left"));
  br_.sendTransform(tf::StampedTransform(arm_u_right, ros::Time::now(), root_, prefix_+"link_u_lower_right"));

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "mpp3h_state_publisher");
  Mpp3hStatePublisher mpp3hstatepublisher;
  ros::spin();

  return 0;
}
