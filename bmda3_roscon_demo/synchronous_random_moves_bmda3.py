#!/usr/bin/env python
#################################################################
##\file
#
# \note
# Copyright (c) 2014 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: motoman_bmda3_support
# \note
# ROS stack name: motoman_experimental
# \note
# ROS package name: motoman_bmda3_support
#
# \author
# Thiago de Freitas, email:tdf@ipa.fhg.de
#
# \date Date of creation: September 2014
#
# \brief
# Some Trajectory Action for synchronous movement on the MOTOMAN bmda3 robot
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################


import roslib; 
roslib.load_manifest('motoman_bmda3_support')
import rospy
import math
import sys
import string
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from sensor_msgs.msg import JointState
from actionlib import simple_action_client
from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryAction

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int64

import random

class synchronous_traj():
  def __init__(self):

    #initializing the ROS node
    rospy.init_node('synchronous_traj_action')

    self.alternate_value = 0.0

  def synchronous_traj_action(self):
    #Generating some "randomized" values for the robot movements, please
    # TODO: check if this value range is suitable for the BMDA3
    
    self.alternate_value = random.uniform(-1.2, 1.2)

    torso_center     = [0.0, 0.0]
    start         = [-1.57, -1.06, 1.45, -0.8, 0.0, 0.0, 0.0]

    front         = [0.70, 1.06, -0.61, 0.35, -0.55, 0.45, 0.0]
    front_flip    = [0.70, 1.06, -0.61, 0.35, 2.59, -0.45, 0.0]


    front2        = [0.20, 1.06, -0.15, 0.35, -0.15, 0.65, 0.0]
    front_flip2   = [0.20, 1.06, -0.15, 0.35, 2.97, -0.65, 0.0]

    #right_shoulder= [0.89, 1.19, -0.50, -1.00, 0.72, 0.00, 0.0]
    right_shoulder= [0.86, 1.38, 0.06, -0.59, 1.06, -1.34, 0.0]
    left_shoulder = [0.66, 1.25, -0.16, -0.45, 0.03, -0.90, 0.0]
    behind_head   = [-1.39, 0.23, -0.55, -1.07, 0.24, 0.0, 0.0]
    right_hip     = [-1.57, -0.92, 2.75, -0.76, 1.63, 0.53, -1.51]
    left_hip      = [1.02, 0.97, -0.19, -0.61, 0.0, 0.0, 0.0]

    pre_back      = [1.45, -1.0, 0.0, -1.35, 0.0, -0.2, 0.0]
    back          = [0.22, -0.78, 2.26, -0.31, -0.41, -0.71, 2.31]

    arm_back      = [3.12, 1.75, -1.87, -1.07, 0.51, -0.54, 0.00]
    arm_up        = [0.76, 1.71, -1.06, -0.37, 0.00, 0.00, 0.00]
    arm_side      = [1.73, 1.12, -1.61, -0.70, 0.34, 0.0, 0.0]
    torso_left    = [0.50, 0.50]
    torso_right   = [-0.50, -0.50]
 

    home      = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    arm_bent  = [-1.36, -1.0, -0.2, -0.6, 0.0, 1.3, 0.0]
    arm_front = [0.7, 0.3, -0.5, -0.5, 0.0, 0.5, -0.2]
    arm_share = [0.0, 0.24, 0.0, -0.5, 0.0, -0.85, 0.0]


    # Getting the current joint_states for each robot group, the controller needs to be checked
    # TODO: Please, check if the torso is also in two controller(FS100) groups for the BMDA3
    #msg_r1 = rospy.wait_for_message("/bmda3/bmda3_r1_controller/joint_states", JointState, 5.0)
    #msg_r2 = rospy.wait_for_message("/bmda3/bmda3_r2_controller/joint_states", JointState, 5.0)
    #msg_b1 = rospy.wait_for_message("/bmda3/bmda3_b1_controller/joint_states", JointState, 5.0)
    #msg_b2 = rospy.wait_for_message("/bmda3/bmda3_b2_controller/joint_states", JointState, 5.0)
    positions=self.get_state()

    
    if(0):
      # iterate over several configurations
      positions=self.get_state()
      self.send_goal(positions, start, start, torso_center)

      positions=self.get_state()
      self.send_goal(positions, start, front, torso_center)

      positions=self.get_state()
      self.send_goal(positions, front, front, torso_center)

      positions=self.get_state()
      self.send_goal(positions, front, front_flip, torso_center)

      positions=self.get_state()
      self.send_goal(positions, front_flip, front_flip, torso_center)

      positions=self.get_state()
      self.send_goal(positions, front_flip, right_shoulder, torso_center)

      positions=self.get_state()
      self.send_goal(positions, left_shoulder, right_shoulder, torso_center)

      positions=self.get_state()
      self.send_goal(positions, behind_head, right_shoulder, torso_center)

      positions=self.get_state()
      self.send_goal(positions, behind_head, behind_head, torso_center)

      positions=self.get_state()
      self.send_goal(positions, behind_head, right_hip, torso_center)

      positions=self.get_state()
      self.send_goal(positions, left_hip, right_hip, torso_center)

      positions=self.get_state()
      self.send_goal(positions, pre_back, right_hip, torso_center)

      positions=self.get_state()
      self.send_goal(positions, back, right_hip, torso_center)

      positions=self.get_state()
      self.send_goal(positions, back, pre_back, torso_center)

      positions=self.get_state()
      self.send_goal(positions, back, back, torso_center)


      # Return to start

      positions=self.get_state()
      self.send_goal(positions, pre_back, pre_back, torso_center)

      positions=self.get_state()
      self.send_goal(positions, start, start, torso_center)

      rospy.sleep(4)

    # Run script
    # run1          = arm_back + arm_up + torso_left
    # run2          = arm_up + arm_back + torso_right

    for i in range(4):
      positions=self.get_state()
      self.send_goal(positions, arm_back, arm_up, torso_right)

      positions=self.get_state()
      self.send_goal(positions, arm_side, arm_side, torso_center)

      positions=self.get_state()
      self.send_goal(positions, arm_up, arm_back, torso_left)

      positions=self.get_state()
      self.send_goal(positions, arm_side, arm_side, torso_center)

    rospy.sleep(2)

    positions=self.get_state()
    self.send_goal(positions, start, start, torso_center)
       

  def send_goal(self, start, left_arm, right_arm, torso):

    #### This is the trajectory action client for synchronized movement for all groups
    bmda3_client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
    bmda3_client.wait_for_server()

    # Creates the goal object to pass to the server
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    # Populates trajectory with joint names.
    goal.trajectory.joint_names = ['left_joint_1_s','left_joint_2_l','left_joint_3_e','left_joint_4_u', 'left_joint_5_r', 'left_joint_6_b', 'left_joint_7_t','right_joint_1_s','right_joint_2_l','right_joint_3_e','right_joint_4_u', 'right_joint_5_r', 'right_joint_6_b', 'right_joint_7_t','torso_joint_b1','torso_joint_b2']

    # First trajectory point
    point_index = 0
    # This should always be the current position,as required by the MOTOROS side
    point1 = trajectory_msgs.msg.JointTrajectoryPoint()
    point2 = trajectory_msgs.msg.JointTrajectoryPoint()
    goal.trajectory.points = [point1, point2]
    #point1.positions = msg_r1.position+msg_r2.position + msg_b1.position + msg_b2.position
    point1.positions = start
    point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point1.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point1.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    goal.trajectory.points[point_index] = point1
    goal.trajectory.points[point_index].time_from_start = rospy.Duration(0.0)

    # Second trajectory point:
    point_index += 1
    # This is the actual goal position for the robot, in this case, the random
    # generated position
    point2.positions = left_arm + right_arm + torso
    point2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point2.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point2.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    goal.trajectory.points[point_index] = point2
    # TODO: the rospy.Duration(...) implies how faster the robot moves,
    # please check that for the BMDA3. The max for the SDA10F without controller
    # errors was rospy.Duration(0.5)
    goal.trajectory.points[point_index].time_from_start = rospy.Duration(1.0)

    goal.trajectory.header.stamp = rospy.Time.now()

    bmda3_client.send_goal_and_wait(goal)

    #rospy.sleep(2.0)
    
  def get_state(self):

    msg_r1 = rospy.wait_for_message("/bmda3/bmda3_r1_controller/joint_states", JointState, 5.0)
    msg_r2 = rospy.wait_for_message("/bmda3/bmda3_r2_controller/joint_states", JointState, 5.0)
    msg_b1 = rospy.wait_for_message("/bmda3/bmda3_b1_controller/joint_states", JointState, 5.0)
    msg_b2 = rospy.wait_for_message("/bmda3/bmda3_b2_controller/joint_states", JointState, 5.0)
    current_pos = msg_r1.position+msg_r2.position + msg_b1.position + msg_b2.position
    #positions=self.get_state()
    #return positions
    return current_pos


if __name__=='__main__':
  sync_traj = synchronous_traj()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    sync_traj.synchronous_traj_action()
    r.sleep()



