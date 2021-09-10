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
from geometry_msgs.msg import Pose, PoseStamped

class RlActionServerclass:
  def __init__(self):
    _feedback = RLActionServerActionFeedback()
    _result = RLActionServerActionResult()

    self._name = 'rl_actions'
    self.server = actionlib.SimpleActionServer('rl_actions', RLActionServerAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    
    self.server.set_succeeded()