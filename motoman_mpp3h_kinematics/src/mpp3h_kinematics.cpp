#include "motoman_mpp3h_kinematics/mpp3h_kinematics.h"

inline tf::Transform createTf (tf::Vector3 point,
                               double roll,
                               double pitch,
                               double yaw) {

    return tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), point);
}

tf::Matrix3x3 getOrientationOfAxisBetween(const tf::Transform &parent,
                                          const tf::Vector3   &child){

    tf::Vector3 axis = child - parent.getOrigin();
    axis.normalize();
    tf::Matrix3x3 mat(parent.getRotation());
    tf::Vector3 zz = mat.getColumn(2);
    tf::Vector3 yy = axis.cross(zz);
    yy.normalize();
    zz = axis.cross(yy);

    tf::Matrix3x3 rotation(
         zz.getX(), axis.getX(), yy.getX(),
         zz.getY(), axis.getY(), yy.getY(),
         zz.getZ(), axis.getZ(), yy.getZ());

    return rotation;
}

bool Mpp3hKinematics::calcSingleAng(tf::Vector3 cartesian_pt,
                                    double ang,
                                    double &theta) const {

  cartesian_pt = cartesian_pt.rotate(tf::Vector3(0,0,1), -ang);

  /* Ref: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
      Geometrical parameters e,f of triangle chosen differently */
  double y1 = -dist_motor_to_center_;
  cartesian_pt.setY(cartesian_pt.getY() - dist_armfixt_to_tool_);
  double a = (cartesian_pt.dot(cartesian_pt) + upper_arm_len_*upper_arm_len_ - lower_arm_len_*lower_arm_len_ - y1*y1)/(2*cartesian_pt.getZ());
  double b = (y1 - cartesian_pt.getY())/cartesian_pt.getZ();
  double discr = -(a+b*y1)*(a+b*y1)+upper_arm_len_*(b*b*upper_arm_len_+upper_arm_len_);
  if (discr < 0) return false; /* non-existing point*/
  double yj = (y1 - a*b - sqrt(discr))/(b*b + 1);
  double zj = a + b*yj;
  theta = atan2(-zj,(y1 - yj));
  if (yj>y1) theta += M_PI;

  return true;
}

bool Mpp3hKinematics::inverseKinematics(tf::Vector3 cartesian_pt,
                                        double tool_ang,
                                        boost::array<double,4> &joints) const {

  // Mpp3h-COOS is shifted by 90 deg with respect to kinematic calculations
  cartesian_pt = cartesian_pt.rotate(tf::Vector3(0,0,1), -shift_ang_);
  joints[3] = tool_ang - shift_ang_;

  return calcSingleAng(cartesian_pt, 0.0,          joints[0])  // Calc 1st arm ( 0   deg rotation)
      && calcSingleAng(cartesian_pt, -geo_ang_, joints[1])     // Calc 2nd arm (-120 deg rotation)
      && calcSingleAng(cartesian_pt,  geo_ang_, joints[2]);    // Calc 3th arm (+120 deg rotation)
}

bool Mpp3hKinematics::forwardKinematics(const boost::array<double,4> &joints,
                                        tf::Vector3 &cartesian_pt,
                                        double &tool_ang) const {

  /* Ref: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
    Geometrical parameters e,f of triangle chosen differently */
  double t = (dist_motor_to_center_-dist_armfixt_to_tool_);
  double y1 = -(t + upper_arm_len_*cos(joints[0]));
  double z1 = -upper_arm_len_*sin(joints[0]);
  double y2 = (t + upper_arm_len_*cos(joints[1]))*sin(M_PI/6);
  double x2 = y2*tan(M_PI/3);
  double z2 = -upper_arm_len_*sin(joints[1]);
  double y3 = (t + upper_arm_len_*cos(joints[2]))*sin(M_PI/6);
  double x3 = -y3*tan(M_PI/3);
  double z3 = -upper_arm_len_*sin(joints[2]);
  double dnm = (y2-y1)*x3-(y3-y1)*x2;
  double w1 = y1*y1 + z1*z1;
  double w2 = x2*x2 + y2*y2 + z2*z2;
  double w3 = x3*x3 + y3*y3 + z3*z3;
  // x = (a1*z + b1)/dnm
  double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
  // y = (a2*z + b2)/dnm;
  double a2 = -(z2-z1)*x3+(z3-z1)*x2;
  double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
  // a*z^2 + b*z + c = 0
  double a = a1*a1 + a2*a2 + dnm*dnm;
  double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - lower_arm_len_*lower_arm_len_);
  double discr = b*b - (double)4.0*a*c;
  if (discr < 0) return false; // non-existing point
  cartesian_pt.setZ(-(double)0.5*(b+sqrt(discr))/a);
  cartesian_pt.setX(-(a1*cartesian_pt.getZ() + b1)/dnm);
  cartesian_pt.setY((a2*cartesian_pt.getZ() + b2)/dnm);

  // Mpp3h-COOS is shifted by 90 deg with respect to kinematic calculations
  tool_ang = joints[3] + shift_ang_;
  cartesian_pt = cartesian_pt.rotate(tf::Vector3(0,0,1), shift_ang_);
  return true;
}

void Mpp3hKinematics::calcSingleArmTransform(double ang,
                                             double joint_value,
                                             const tf::Transform &tool,
                                             tf::Transform &lower_arm_left,
                                             tf::Transform &lower_arm_right) const {

  tf::Transform upper_arm, lower_arm;
  tf::Vector3 upper_arm_v = tf::Vector3(0.0, -dist_motor_to_center_, 0.0);

  // Get position of the lower arm
  upper_arm_v = upper_arm_v.rotate(tf::Vector3(0,0,1),ang);
  upper_arm = createTf(upper_arm_v, ang, -M_PI/2, M_PI); // Angles correspond to urdf
  upper_arm.setRotation(upper_arm.getRotation() * tf::createQuaternionFromRPY(0, 0, joint_value));
  lower_arm = createTf(tf::Vector3(0.0, upper_arm_len_, 0.0), 0.0, -M_PI/2, -M_PI/3); // Angles correspond to urdf
  lower_arm = upper_arm*lower_arm;

  // Get orientation of the lower arm by the position of the lower arm and the tool(forward kinematic)
  tf::Vector3 tool_fixture_offset = tf::Vector3(dist_armfixt_to_tool_*sin(ang),-dist_armfixt_to_tool_*cos(ang),0);
  tf::Matrix3x3 orientation = getOrientationOfAxisBetween(lower_arm, tool.getOrigin()+tool_fixture_offset);
  lower_arm_left.setBasis(orientation);
  lower_arm_right.setBasis(orientation);

  // Lower arms are two links arranged in a parallelogram
  tf::Vector3 parallel_offset = tf::Vector3(dist_lower_arm_links_*cos(ang), dist_lower_arm_links_*sin(ang),0);
  lower_arm_left.setOrigin(lower_arm.getOrigin()-parallel_offset);
  lower_arm_right.setOrigin(lower_arm.getOrigin()+parallel_offset);
}

bool Mpp3hKinematics::getTransforms(const boost::array<double,4> &joints,
                                    tf::Transform &tool,
                                    tf::Transform &arm_s_left,
                                    tf::Transform &arm_s_right,
                                    tf::Transform &arm_l_left,
                                    tf::Transform &arm_l_right,
                                    tf::Transform &arm_u_left,
                                    tf::Transform &arm_u_right) const {

    double tool_ang;
    tf::Vector3 cartesian_pt;
    if (!forwardKinematics(joints, cartesian_pt, tool_ang)) {
      ROS_WARN("Could not calculate FK for given pose");
      return false;
    }
    tool = createTf(cartesian_pt, 0.0, 0.0, M_PI/6); // Angles correspond to urdf

    // Mpp3h-COOS is shifted by 90 deg with respect to kinematic calculations
    tool.setRotation(tool.getRotation() * tf::createQuaternionFromRPY(0, 0, shift_ang_));
    calcSingleArmTransform(shift_ang_,          joints[0], tool, arm_s_left, arm_s_right);
    calcSingleArmTransform(shift_ang_-geo_ang_, joints[1], tool, arm_l_left, arm_l_right);
    calcSingleArmTransform(shift_ang_+geo_ang_, joints[2], tool, arm_u_left, arm_u_right);

    return true;
}
