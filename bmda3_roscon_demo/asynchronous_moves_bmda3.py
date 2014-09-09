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
SCALE_MOTION = 0.1


class synchronous_traj():
  def __init__(self):

    #initializing the ROS node
    rospy.init_node('synchronous_traj_action')

    self.alternate_value = 0.0

  def synchronous_traj_action(self):
    #Generating some "randomized" values for the robot movements, please
    # TODO: check if this value range is suitable for the BMDA3
    
    self.alternate_value = random.uniform(-1.2, 1.2)

    torso_center  = [0.0, 0.0]
    start         = [-1.57, -1.06, 1.45, -0.8, 0.0, 0.0, 0.0]
    front         = [0.70, 1.06, -0.61, 0.35, -0.55, 0.45, 0.0]
    front_flip    = [0.70, 1.06, -0.61, 0.35, 2.59, -0.45, 0.0]
    right_shoulder= [0.86, 1.38, 0.06, -0.59, 1.06, -1.34, 0.0]
    left_shoulder = [0.66, 1.25, -0.16, -0.45, 0.03, -0.90, 0.0]
    behind_head   = [-1.39, 0.23, -0.55, -1.07, 0.24, 0.0, 0.0]
    right_hip     = [-1.57, -0.92, 2.75, -0.76, 1.63, 0.53, -1.51]
    left_hip      = [1.02, 0.97, -0.19, -0.61, 0.0, 0.0, 0.0]
    pre_back      = [1.45, -1.0, 0.0, -1.35, 0.0, -0.2, 0.0]
    back          = [0.22, -0.78, 2.26, -0.31, -0.41, -0.71, 2.31]

 

    # iterate over several configurations
    
    self.evaluate_goal(start, [], torso_center)

    
    self.evaluate_goal([], start, torso_center)

    
    self.evaluate_goal([], front, torso_center)

    
    self.evaluate_goal(front, [], torso_center)

    
    self.evaluate_goal([], front_flip, torso_center)

    
    self.evaluate_goal(front_flip, [], torso_center)

    
    self.evaluate_goal([], right_shoulder, torso_center)

    
    self.evaluate_goal(left_shoulder, [], torso_center)

    
    self.evaluate_goal(behind_head, [], torso_center)

    
    self.evaluate_goal([], behind_head, torso_center)

    
    self.evaluate_goal([], right_hip, torso_center)

    
    self.evaluate_goal(left_hip, [], torso_center)

    
    self.evaluate_goal(pre_back, [], torso_center)

    
    self.evaluate_goal(back, [], torso_center)

    
    self.evaluate_goal([], pre_back, torso_center)

    
    self.evaluate_goal([], back, torso_center)

    # Return to start
    
    self.evaluate_goal([], pre_back, torso_center)

    
    self.evaluate_goal([], start, torso_center)

    
    self.evaluate_goal(pre_back, [], torso_center)

    
    self.evaluate_goal(start, [], torso_center)

    rospy.sleep(4)

       

  def evaluate_goal(self, left_arm, right_arm, torso):
  
    if len(left_arm) == 7:
      self.send_goal("r1", left_arm)
    if len(right_arm)==7:
      self.send_goal("r2", right_arm)
    
    self.send_goal("b1", torso[0])
    
    self.send_goal("b2", torso[1])

  def send_goal(self,group, goal_positions):
    ## This generates a subscriber to the current position for the first controller
    # group. Also the joint_trajectory_action now corresponds to the trajectory
    # control for this group point_indexividually.
    msg = rospy.wait_for_message("/bmda3/bmda3_"+group+"_controller/joint_states", JointState, 5.0)
    bmda3_client = actionlib.SimpleActionClient("/bmda3/bmda3_"+group+"_controller/joint_trajectory_action", FollowJointTrajectoryAction)
    bmda3_client.wait_for_server()

    # Creates the goal object to pass to the server
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    #######################################
    # Independent trajectory for the left arm
    #######################################
    # Populates trajectory with joint names.
    if group == "r1":
      goal.trajectory.joint_names = ['left_joint_1_s','left_joint_2_l','left_joint_3_e','left_joint_4_u', 'left_joint_5_r', 'left_joint_6_b', 'left_joint_7_t']
    elif group =="r2":
      goal.trajectory.joint_names = ['right_joint_1_s','right_joint_2_l','right_joint_3_e','right_joint_4_u', 'right_joint_5_r', 'right_joint_6_b', 'right_joint_7_t']
    elif group =="b1":
      goal.trajectory.joint_names = ['torso_joint_b1']
    elif group == "b2":
      goal.trajectory.joint_names = ['torso_joint_b2']
    else:
      return

    # First trajectory point - this should always be the actual position
    # Positions
    point_index = 0
    point1 = trajectory_msgs.msg.JointTrajectoryPoint()
    point2 = trajectory_msgs.msg.JointTrajectoryPoint()
    goal.trajectory.points = [point1, point2]

    point1.positions = msg.position
    point1.velocities = [0.0]*len(goal.trajectory.joint_names)
    point1.accelerations = [0.0]*len(goal.trajectory.joint_names)
    point1.effort = [0.0]*len(goal.trajectory.joint_names)
    goal.trajectory.points[point_index] = point1
    goal.trajectory.points[point_index].time_from_start = rospy.Duration(0.0)

    # Second trajectory point - the goal position
    # Positions
    point_index += 1
    if group == "b1" or group=="b2":
      point2.positions = [goal_positions]
    else:
      point2.positions = goal_positions
    point2.positions = [x*SCALE_MOTION for x in point2.positions]
    point2.velocities = [0.0]*len(goal.trajectory.joint_names)
    point2.accelerations = [0.0]*len(goal.trajectory.joint_names)
    point2.effort = [0.0]*len(goal.trajectory.joint_names)
    goal.trajectory.points[point_index] = point2
    # TODO: the rospy.Duration(...) implies how faster the robot moves,
    # please check that for the BMDA3. The max for the SDA10F without controller
    # errors was rospy.Duration(0.5)
    goal.trajectory.points[point_index].time_from_start = rospy.Duration(1.0)
    goal.trajectory.header.stamp = rospy.Time.now()
    
    if group == "b1" or group=="b2":
      bmda3_client.send_goal(goal)
      rospy.sleep(0.4)
    else:
      bmda3_client.send_goal_and_wait(goal, rospy.Duration(4.0))
      
if __name__=='__main__':
  sync_traj = synchronous_traj()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    sync_traj.synchronous_traj_action()
    r.sleep()



