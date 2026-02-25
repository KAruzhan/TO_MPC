#To launch ur5 using ROS2:

colcon build
source install/setup.bash
ros2 run ur_ros2_driver ur_driver

# and wait until "DEBUG: Realtime port: Got connection" message

ros2 run pb_ur5_sim my_node_robot 

ros2 run sim_utils human_motion_player

# run mppi controller
ros2 run mppi_controller mppi_rh

ros2 topic pub --once /start std_msgs/msg/String data:\ \'\'\ 

#################################################################

colcon build
source install/setup.bash
ros2 run ur_ros2_driver ur_driver

# and wait until "DEBUG: Realtime port: Got connection" message

ros2 run pb_ur5_sim my_node_robot 

ros2 run mppi_controller mppi_robot




# verify connection
ros2 topic echo /joint_states

##############################

# To send commands from terminal:
ros2 topic pub /my_UR5/velocity_command std_msgs/msg/Float64MultiArray "data:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0" -r 100

##############################

#Examples of how to control UR5:

python3 test_acceleration.py #using UR_Script
#or
python3 joint_tuner.py #Using joint veloicty

