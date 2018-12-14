source devel/setup.bash


rosrun baxter_tools enable_robot.py -e &
# rosrun intera_interface enable_robot.py -e &

rosrun baxter_tools camera_control.py -c head_camera
# rosrun baxter_tools camera_control.py -o right_hand_camera
rosrun baxter_tools camera_control.py -o right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera
# rosrun planning start_sawyer_cam.py &

roslaunch ar_track_alvar baxter_right_hand_track.launch &
roslaunch ar_track_alvar baxter_left_hand_track.launch &
# roslaunch ar_track_alvar sawyer_right_hand_track.launch &

rosrun baxter_interface joint_trajectory_action_server.py &
# rosrun intera_interface joint_trajectory_action_server.py &

roslaunch baxter_moveit_config baxter_grippers.launch &
# roslaunch planning baxter_grippers.launch &

# roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true  & #electric_gripper:=false 
rosrun rviz rviz &
