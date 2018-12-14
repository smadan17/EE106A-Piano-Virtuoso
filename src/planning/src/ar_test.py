#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np
import yaml
import os

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Vector3
from path_planner import PathPlanner
from baxter_interface import Limb

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from intera_interface import Limb

cwd = os.path.dirname(__file__)

def play_song(note_pos):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

    z_offset = 0.05
    ##
    ## Add the obstacle to the planning scene here
    ##

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    # # Note 1 for testing
    # note1 = Vector3()
    # note1.x = 0.6
    # note1.y = -0.3
    # note1.z = 0.1

    # # Note 2 for testing
    # note2 = Vector3()
    # note2.x = 0.6
    # note2.y = -0.25
    # note2.z = 0.1


    # while not rospy.is_shutdown():

    # Iterate through all note positions for the song
    for i, pos in enumerate(note_pos):

        # Loop until that position is reached
        while not rospy.is_shutdown():
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = pos.x
                goal_1.pose.position.y = pos.y
                goal_1.pose.position.z = pos.z

                #Orientation as a quaternion, facing straight down
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_1, list())

                # raw_input("Press <Enter> to move the right arm to goal pose {}: ".format(i))
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')

  
    # Note 1 for testing
    note1 = Vector3()
    note1.x = 0.885
    note1.y = 0.098
    note1.z = 0.05

# [0.885, 0.098, -0.205] ar
# [0.817, 0.195, -0.076] l tip
# Translation: [0.819, 0.195, -0.056]: l gripper base

    # Execute path
    play_song([note1])
