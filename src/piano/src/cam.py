#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
from geometry_msgs.msg import PoseStamped, Vector3
from path_planner import PathPlanner

import cv2
import yaml
import os
import piano

cwd = os.getcwd() + "/"

with open(cwd + "cfg.yml", "r") as ymlfile:
    cfg = yaml.load(ymlfile)


# Instantiate CvBridge
bridge = CvBridge()

# Output file
output_file = cwd + cfg['inputDir'] + cfg['imgName'] + cfg['imgFormat']

def take_image():
    # Define your image topic
    rospy.init_node('cam')

    image_topic = "/cameras/left_hand_camera/image"

    s = raw_input("Press any character to take an image:\n")

    img = rospy.wait_for_message(image_topic, Image)

    print("Took an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        print(output_file)
        cv2.imwrite(output_file, cv2_img)

def main():
    #Set the inital pose
    while not rospy.is_shutdown():
        try:
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            #x, y, and z position
            goal_1.pose.position.x = 0.927
            goal_1.pose.position.y = 0.178
            goal_1.pose.position.z = 0.160

            #Orientation as a quaternion, facing straight down
            goal_1.pose.orientation.x = -0.038
            goal_1.pose.orientation.y = 0.995
            goal_1.pose.orientation.z = -0.017
            goal_1.pose.orientation.w = 0.086

            plan = planner.plan_to_pose(goal_1, list())

            # raw_input("Press <Enter> to move the right arm to goal pose {}: ".format(i))
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break

    # Take picture of piano
    take_image()


if __name__ == '__main__':
    main()