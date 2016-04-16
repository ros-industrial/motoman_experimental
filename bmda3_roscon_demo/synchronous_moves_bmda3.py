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

# Scale down motion range, 1.0 is normal range, 2.0 cuts motion movement in half
SCALE_MOTION = 0.2

class synchronous_traj():
  def __init__(self):

    #initializing the ROS node
    rospy.init_node('synchronous_traj_action')

    self.bmda3_client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
    self.alternate_value = 0.0


    self.traj = control_msgs.msg.FollowJointTrajectoryGoal()

    # Creates the goal object to pass to the server
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    # Populates trajectory with joint names.
    self.traj.trajectory.joint_names = ['left_joint_1_s','left_joint_2_l','left_joint_3_e','left_joint_4_u', 'left_joint_5_r', 'left_joint_6_b', 'left_joint_7_t','right_joint_1_s','right_joint_2_l','right_joint_3_e','right_joint_4_u', 'right_joint_5_r', 'right_joint_6_b', 'right_joint_7_t','torso_joint_b1','torso_joint_b2']

    # First trajectory point
    self.point_index = 0
    # This should always be the current position,as required by the MOTOROS side
    #point1 = trajectory_msgs.msg.JointTrajectoryPoint()

    #start_position=self.get_state()
    #point1.positions = msg_r1.position+msg_r2.position + msg_b1.position + msg_b2.position
    #point1.positions = start_position
    #point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    #point1.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    #point1.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    #self.traj.points = [point1]

    #self.traj.trajectory.points[point_index].time_from_start = rospy.Duration(0.0)



  def synchronous_traj_action(self):
    #Generating some "randomized" values for the robot movements, please
    # TODO: check if this value range is suitable for the BMDA3
    
    self.alternate_value = random.uniform(-1.2, 1.2)

    torso_center  = [0.0, 0.0]
    start         = [1.57, 1.06, -1.45, -0.8, 0.0, 0.0, 0.0]

    arm_back      = [3.12, 1.75, -1.87, -1.07, 0.51, -0.54, 0.00]
    arm_up        = [0.76, 1.71, -1.06, -0.37, 0.00, 0.00, 0.00]
    arm_side      = [1.73, 1.12, -1.61, -0.70, 0.34, 0.0, 0.0]
    torso_left    = [0.50, 0.50]
    torso_right   = [-0.50, -0.50]
 

    #positions=self.get_state()
    #self.send_goal(positions, arm_side, arm_side, torso_center)

   

    for i in range(1):
      positions=self.get_state()
      self.send_goal(positions, arm_back, arm_up, torso_right)

      positions=self.get_state()
      self.send_goal(positions, arm_side, arm_side, torso_center)

      positions=self.get_state()
      self.send_goal(positions, arm_up, arm_back, torso_left)

      positions=self.get_state()
      self.send_goal(positions, arm_side, arm_side, torso_center)

    positions=self.get_state()
    self.bmda3_client.send_goal_and_wait(positions, start, start, torso_center)
    
    rospy.sleep(2)
   
  def append_goal(self, target_position):

    # First trajectory point
    self.point_index += 1
    # This should always be the current position,as required by the MOTOROS side
    point1 = trajectory_msgs.msg.JointTrajectoryPoint()

    #point1.positions = msg_r1.position+msg_r2.position + msg_b1.position + msg_b2.position
    point1.positions = target_position
    point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point1.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point1.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point1.time_from_start = rospy.Duration(self.point_index)


    self.traj.points.append(goal)

  def send_goal(self, start, left_arm, right_arm, torso):

    #### This is the trajectory action client for synchronized movement for all groups
    #bmda3_client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for service")
    self.bmda3_client.wait_for_server()
    rospy.loginfo("DONE Waiting for service")

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
    point2.positions =  [x*SCALE_MOTION for  x in  point2.positions]
    point2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point2.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point2.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    goal.trajectory.points[point_index] = point2
    # TODO: the rospy.Duration(...) implies how faster the robot moves,
    # please check that for the BMDA3. The max for the SDA10F without controller
    # errors was rospy.Duration(0.5)
    goal.trajectory.points[point_index].time_from_start = rospy.Duration(1.0)

    goal.trajectory.header.stamp = rospy.Time.now()

    rospy.loginfo("Sending Goal")
    self.bmda3_client.send_goal_and_wait(goal)
    #r = rospy.Rate(.25)
    #r.sleep()
    rospy.loginfo("Done sending goal")

    #rospy.sleep(2.0)
    
  def get_state(self):
    rospy.loginfo("Get start state")
    msg_r1 = rospy.wait_for_message("/bmda3/bmda3_r1_controller/joint_states", JointState, 2.0)
    msg_r2 = rospy.wait_for_message("/bmda3/bmda3_r2_controller/joint_states", JointState, 2.0)
    msg_b1 = rospy.wait_for_message("/bmda3/bmda3_b1_controller/joint_states", JointState, 2.0)
    msg_b2 = rospy.wait_for_message("/bmda3/bmda3_b2_controller/joint_states", JointState, 2.0)
    current_pos = msg_r1.position+msg_r2.position + msg_b1.position + msg_b2.position
    #current_pos = rospy.wait_for_message("/joint_states", JointState, 5.0)
    rospy.loginfo("Get start state DONE")
    return current_pos
    #return current_pos.position


if __name__=='__main__':
  sync_traj = synchronous_traj()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    sync_traj.synchronous_traj_action()
    r.sleep()



