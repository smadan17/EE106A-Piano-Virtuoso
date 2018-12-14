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
import piano
# from src.cam import take_image
from src.piano import map_keys_to_pixels
from baxter_interface import gripper as robot_gripper
import tf 

import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from sensor_msgs.msg import Image
# from intera_interface import Limb

cwd = os.getcwd() + "/"
with open(cwd + "cfg.yml", "r") as ymlfile:
    cfg = yaml.load(ymlfile)

listen_topic = cfg["positionTopic"]

def play_song(note_pos, pose):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    waypoint_offset = 0.05
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


    # note_pos = [note1, note2]

    new_note_pos = []

    # add waypoints above the note to ensure the note is struck properly
    for note in note_pos:
        new_note = Vector3()
        new_note.x = note.x
        new_note.y = note.y
        new_note.z = note.z + waypoint_offset

        new_note_pos.append(new_note)
        new_note_pos.append(note)
        new_note_pos.append(new_note)

    # while not rospy.is_shutdown():
    # Iterate through all note positions for the song
    for i, pos in enumerate(new_note_pos):
        print(i, pos)
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
                goal_1.pose.orientation.x = 0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0
                goal_1.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_1, list())

                # raw_input("Press <Enter> to move the right arm to goal pose {}: ".format(i))
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

def take_image():
    # piano_cwd = os.path.dirname(__file__) + "/../../piano/src/"
    piano_cwd = os.getcwd() + "/"

    jsonFile =  piano_cwd + cfg['outputDir'] + cfg['imgName'] + ".json"
    imgFile = piano_cwd + cfg['inputDir'] + cfg['imgName'] + cfg['imgFormat']

    # Instantiate CvBridge
    bridge = CvBridge()

    # Output file
    output_file = piano_cwd + cfg['inputDir'] + cfg['imgName'] + cfg['imgFormat']
    # print(output_file)

    image_topic = "/cameras/right_hand_camera/image"

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
        cv2.imwrite(output_file, cv2_img)

    return output_file

# def get_homography_matrix(xy_points, uv):
#     # print(xy_points)
#     # print(uv)
#     u = uv[0]
#     v = uv[1]
#     A = np.zeros((2*len(xy_points), 8))

#     for i, point in enumerate(xy_points):
#         row1 = A[2*i]
#         row2 = A[2*i + 1]

#         row1[0] = point[0]
#         row1[1] = point[1]
#         row1[2] = 1
#         row1[6] = -u[i] * point[0] 
#         row1[7] = -u[i] * point[1]


#         row2[3] = point[0]
#         row2[4] = point[1]
#         row2[5] = 1
#         row2[6] = -v[i] * point[0] 
#         row2[7] = -v[i] * point[1]

#         A[2*i] = row1
#         A[2*i+1] = row2

#     b = []

#     for i in range(len(u)):
#         b.append(u[i])
#         b.append(v[i])

#     b = np.array(b).T
#     A_inv = np.linalg.inv(A)
#     x = np.dot(A_inv, b)
#     x = np.append(x, [1])
#     H = np.reshape(x, (3, 3))
#     # print("x: ", x)
#     # print("H: ", H)

#     return H  

# def do_homography(img_filepath):
#     # Finds the pixel coordinates for the centers of each AR tag
#     img = cv2.imread(img_filepath)
#     # img = cv2.resize(img, (960, 540))
#     # # cv2.imshow('image', img)
#     # # cv2.waitKey(0)
#     # # cv2.destroyAllWindows()

#     tag_binary_matrices = {
#         4: np.array([
#             [0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 0, 1, 0, 1, 0],
#             [0, 0, 1, 1, 1, 0, 0],
#             [0, 0, 1, 1, 1, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0],
#         ]),
#         9: np.array([
#             [0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 0, 1, 0, 1, 0],
#             [0, 1, 1, 1, 0, 0, 0],
#             [0, 1, 1, 1, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0],
#         ]),
#         0: np.array([
#             [0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 0, 1, 0, 1, 0],
#             [0, 1, 1, 1, 1, 1, 0],
#             [0, 1, 1, 1, 1, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0],
#         ]),
#         1: np.array([
#             [0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 1, 0, 1, 1, 0],
#             [0, 1, 0, 1, 0, 1, 0],
#             [0, 0, 0, 1, 1, 0, 0],
#             [0, 1, 1, 1, 0, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0],
#         ])
#     }

#     # tag_bytelists = []

#     # for k, v in tag_binary_matrices.items():
#     #     tag_bytelists.append(aruco.Dictionary_getByteListFromBits(v.flatten()))

#     # marker_dict = aruco.custom_dictionary(len(tag_bytelists), 7)
#     # marker_dict.bytesList = np.array(tag_bytelists)
#     # # marker_dict.maxCorrectionBits = 10
#     # # img1 = aruco.drawMarker(marker_dict, 0, 500)
#     # # cv2.imwrite("mtest.jpg", img1
#     # #parameters = aruco.DetectorParameters_create()
#     # #marker_dict = aruco.Dictionary_get(aruco.DICT__250)
#     # corners, ids, rejectedImgPoints = aruco.detectMarkers(img, marker_dict)
#     # print(corners)

#     # id_to_center = {}
#     # for ind, corner_set in enumerate(corners):
#     #     #corner_set is 1, 4, 2
#     #     corner1 = corner_set[0, 0, :]
#     #     corner2 = corner_set[0, 1, :]
#     #     corner3 = corner_set[0, 2, :]
#     #     corner4 = corner_set[0, 3, :]

#     #     corner1 = np.expand_dims(corner1, axis=1)
#     #     corner2 = np.expand_dims(corner2, axis=1)
#     #     corner3 = np.expand_dims(corner3, axis=1)
#     #     corner4 = np.expand_dims(corner4, axis=1)

#     #     axis1 = corner3 - corner1
#     #     axis2 = corner2 - corner4

#     #     print(axis1.shape)
#     #     A = np.hstack((axis1, -axis2))
#     #     b = corner4 - corner1

#     #     x = np.linalg.solve(A, b)

#     #     square_center = corner4 + x[1][0]*axis2
#     #     red = [0,0,255]
#     #     # Change one pixel
#     #     x = int(round(square_center[0][0]))
#     #     y = int(round(square_center[1][0]))
#     #     #print(x, y)

#     #     id_to_center[ids[ind]] = [x, y]
#     #     cv2.circle(img,(x, y), 4, red, -1)
#     #     #img[x,y]=red

#     # img_with_markers = aruco.drawDetectedMarkers(img, corners)
#     # #cv2.imshow('image', img_with_markers) 
#     # #cv2.waitKey(0)
#     # #cv2.destroyAllWindows()
#     # cv2.imwrite("aruco_centers.jpg", img_with_markers)

#     id_to_center= {4: [483, 701], 9: [752, 698], 1: [750, 431], 0: [510, 416]}
    
#     # Use ar_track_alvar to find postion of tag centers
#     listener = tf.TransformListener()

#     id_to_coords = {}
#     for key in tag_binary_matrices:
#         print(key)
#         listener.waitForTransform('/reference/base', '/ar_marker_' + str(key), rospy.Time(0), rospy.Duration(2)) # wait for tf to be generated
#         (trans,rot) = listener.lookupTransform('/reference/base', '/ar_marker_' + str(key), rospy.Time(0))
#         id_to_coords[key] = trans
#     print(id_to_coords)

#     #xy points should have format [(x1, y1), ...] (real world coords)
#     #uv should have formt (image coords)
#     #   | u1 u2 u3 u4 |
#     #   | v1 v2 v3 v4 |
#     # H maps from world to image
#     xy_points = []
#     uv_points = []
#     z_sum = 0
#     for ID in id_to_coords.keys():
#         uv_points.append(id_to_center[ID])
#         trans = id_to_coords[ID]
#         xy_points.append((trans[0], trans[1]))
#         z_sum += trans[2]
#     uv = np.array(uv_points).T
#     H = get_homography_matrix(xy_points, uv)
#     z_avg = z_sum/len(id_to_coords)

#     return H, z_avg341981573374605, 0.03995705298447

# def analyze_piano(img_path):
#     # take_image()

#     # Map keys to pixel coordinates    
#     keys_to_centers = map_keys_to_pixels()
#     #print(keys_to_centers)

#     H, z_empirical = do_homography(img_path)
#     H_inv = np.linalg.inv(H)

#     # Map keys to 3d coordinates
#     keys_to_coords = {}
#     for key, center in keys_to_centers.items():
#         homo_center = np.append(center, [1])
#         homo_pos = np.dot(H_inv, homo_center)
        
#         homo_pos[2] = z_empirical
#         keys_to_coords[key] = homo_pos 

#     return keys_to_coords

def setup_gripper():
     # Set up the gripper
    gripper = robot_gripper.Gripper('left')

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating Gripper...')
    gripper.calibrate()

    #Close the right gripper
    print('Closing Gripper...')
    gripper.close()

def create_note(x, y, z):
    note1 = Vector3()
    note1.x = x
    note1.y = y
    note1.z = z + 0.08
    return note1

def get_transform_g(frame1, frame2):
    listener = tf.TransformListener()

    listener.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(2)) # wait for tf to be generated
    (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
    # print("tr", trans, rot)

    R = tf.transformations.quaternion_matrix(rot)
    euler = tf.transformations.euler_from_quaternion(rot) #roll, pitch, yaw

    R[0][3] = trans[0]
    R[1][3] = trans[1]
    R[2][3] = trans[2]

    return R


def make_note_dict():
    #Use ar_track_alvar to find postion of tag centers
    ar_marker_key = 4
    # g_base_left_hand = get_transform_g('/reference/base', '/reference/right_hand_camera')
    # g_left_hand_ar = get_transform_g('/reference/right_hand_camera', '/ar_marker_' + str(ar_marker_key))
    # g_base_ar = g_base_left_hand.dot(g_left_hand_ar)

    g_base_ar = get_transform_g('/reference/base', '/ar_marker_' + str(ar_marker_key))

    R_sum = g_base_ar[0:3][0:3]
    euler = tf.transformations.euler_from_matrix(R_sum, axes='sxyz')

    return g_base_ar, euler


    


def convert_song_to_coords(str_notes, notes_to_coords_map):
    vector_coords = []
    for note in str_notes:
        coord = notes_to_coords_map.get(note, None) #TODO CHANGE THIS
        vector_coord = Vector3()
        vector_coord.x = coord[0]
        vector_coord.y = coord[1]
        vector_coord.z = coord[2]
        vector_coords.append(vector_coord)

    return vector_coords

def make_key_coords():
    # Maps keys (strings) to positions in the ar tag frame
    off_low = -1.8
    ar_tag_offset = np.array([-20, off_low])
    key_coords_dict = {}

    # WHITE KEYS
    note_names = ["C", "D", "E", "F", "G", "A", "B"]
    note_names = note_names[::-1]

    key_coords_dict["C3"] = ar_tag_offset + np.array([-2.3, 0])
    for i in range(21):
        note_name = note_names[i % 7] + str(2 - (i//7))
        #print(note_name)

        key_coords_dict[note_name] = ar_tag_offset + (i+1)*np.array([-2.3, 0])

    # BLACK KEYS
    # A sharp
    sharp_names = ["A", "G", "F", "D", "C"]
    for j in range(15):
        octave_num = 2 - j//5
        note_name = sharp_names[j%5] + str(octave_num) + "#"
        #print(note_name)
        if (j % 5) <= 2:
            key_coords_dict[note_name] = key_coords_dict["B" + str(octave_num)] + np.array([-(j % 5 + 1)*2.1, 0]) + np.array([-1.0, -2 * off_low])
        else:
            key_coords_dict[note_name] = key_coords_dict["E" + str(octave_num)] + np.array([-(j % 5 + 1)*2.1, 0]) + np.array([-1.0, -2 * off_low])
    
    return key_coords_dict

#WARNING!!!!!!!!!!!!!!!: must run following command before this script
# ./run_demo.sh
if __name__ == '__main__':
    rospy.init_node('moveit_node')

    # img_path = cfg['inputDir'] + cfg['imgName'] + cfg['imgFormat']
    # notes_to_coords_map = analyze_piano(img_path)

    # print(notes_to_coords_map)

    # Get and convert input song to 3d coordinates
    # Assumes input song is space separated string

    # g_base_ar, pose = make_note_dict()

    # key_coords_dict = make_key_coords()
    # key_poses_dict = {}
    # for k in key_coords_dict:
    #     x, y = key_coords_dict[k]
    #     key_poses_dict[k] = np.array([[x/100.0, y/100.0, 0, 1]]).T

    # notes = ['C1', 'C1', 'G1', 'G1', 'A1', 'A1', 'G1', 'F1', 'F1', 'E1', 'E1', 'D1', 'D1', 'C1', 'G1', 'G1', 'F1', 'F1', 'E1', 'E1', 'D1', 'G1', 'G1', 'F1', 'F1', 'E1', 'E1', 'D1', 'C1', 'C1', 'G1', 'G1', 'A1', 'A1', 'G1', 'F1', 'F1', 'E1', 'E1', 'D1', 'D1', 'C1']
    # positions = [key_poses_dict[n] for n in notes]
    # vector_coords = []
    # for p in positions:
    #     base_note_pos = g_base_ar.dot(p)
    #     bns = base_note_pos
    #     print("bns", bns)
    #     note1 = create_note(bns[0][0], bns[1][0], g_base_ar[2][3])
    #     vector_coords.append(note1)

    # play_song(vector_coords, pose)



    setup_gripper()








    # str_notes = raw_input("Enter a song: ").split()
    # vector_coords = convert_song_to_coords(str_notes, notes_to_coords_map)
    # print(vector_coords)    
    # Note 1 for testing
    # note1 = Vector3()
    # # note1.x = 0.6
    # # note1.y = -0.3
    # # note1.z = 0.1

    #note1 = create_note(0.68, 0.12, -0.34)
    # note2 = create_note(0.68, 0.2, -0.34)
    # note3 = create_note(0.68, 0.28, -0.34)
    # # Note 2 for testing
    # # note2 = Vector3()
    # # note2.x = 0.6
    # # note2.y = -0.25
    # # note2.z = 0.1

    
    #note1 = create_note(0.410796046312651, 0.09169295150704115, -0.20060929574303682)
    #note1 = create_note(0.809, 0.11, -0.079)
    #note1 = create_note(0.68, 0.12, -0.34)
    #vector_coords = [note1]

    

    # # Execute path
    #play_song(vector_coords)


"""
Remember to:
1) Update physical side length of AR tags in launch
2) Run joint traj action server, moveit node, artrackalvar launch file
3) Remember in rviz to set fixed_frame as base

"""

"""
4:38 PM 
C1 at 
Translation: [0.824, 0.149, -0.147]
- Rotation: in Quaternion [0.990, -0.001, 0.122, -0.074]
            in RPY (radian) [-2.990, -0.243, -0.020]
            in RPY (degree) [-171.301, -13.935, -1.165]
[baxter - http://asimov.local:11311] c111-8 [507] ~/ros_w
"""