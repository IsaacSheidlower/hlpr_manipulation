#!/usr/bin/env python

import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import actionlib
import geometry_msgs.msg
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
from math import pi, floor, ceil, fabs
import numpy as np
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from hlpr_manipulation_utils.msg import RLActionServerActionGoal, RLActionServerActionResult, RLActionServerActionFeedback
from hlpr_manipulation_utils.msg import RLActionServerAction
from geometry_msgs.msg import Pose, PoseStamped, Point

class RlActionServerclass:
  def __init__(self):
    _feedback = RLActionServerActionFeedback()
    _result = RLActionServerActionResult()

    self._name = 'rl_actions'
    self.server = actionlib.SimpleActionServer('rl_actions', RLActionServerAction, self.execute, auto_start=False)
    self.server.start()

    self.collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

    self.arm = ArmMoveIt("j2s7s300_link_base")
    self.grip = Gripper()


  def execute(self, goal):
    """
      Given a message of type geometry_msgs/Pose , move arm to 
      that goal with collision checking. The goal must be similar to 
      the current position as this action server is meant for 
      real time reinforcement learning. The orientation given
      in the Pose msg is more or less ingored.

      goal: geometry_msgs/Pose
    """
    r = rospy.Rate(.1)
    success = True
    self.server.set_succeeded()

    rospy.loginfo('%s: Checking collisions and executing at time %i' % 
                  (self._name, goal.order, rospy.Time.now()))

    goal_pose = np.array([goal.position.x, goal.position.y, goal.postion.z])
    goal_joints = arm.get_IK(goal)

    # check change in pose is small: 

    # get current joints in sorted order
    arm_curr_pose = self.arm.get_current_pose()
    curr_joints = np.array([arm_curr_pose[x] for x in sorted(arm_curr_pose)])
    curr_pose = np.array([self.arm.get_FK()[0].pose.position.x, self.arm.get_FK()[0].pose.position.y,
                          self.arm.get_FK()[0].pose.position.z])

    # check for current collision or goal collision:
    curr_robot_state = arm.state_from_joints(arm_curr_pose)

    eef_change = curr_pose - goal_pose
    joint_change = curr_joints - goal_joints           

    
