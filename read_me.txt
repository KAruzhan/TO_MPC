
# compile
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Documents/acados/lib
export ACADOS_SOURCE_DIR=~/Documents/acados
colcon build --packages-select pb_ur5_sim mpc_to mpc_low
source install/setup.bash

# MPPI no human
ros2 run pb_ur5_sim my_node 
ros2 run mppi_controller mppi_no_human

# MPPI with human
ros2 run pb_ur5_sim my_node_with_human 
ros2 run sim_utils human_motion_player
ros2 run mppi_controller mppi_controller
ros2 topic pub --once /start std_msgs/msg/String data:\ \'\'\ 


## Read me for MPC

# 1) Start pyBullet
ros2 run pb_ur5_sim my_node_with_human 

# 2) start human position reader (reads sphers position form .csv)
ros2 run sim_utils human_motion_player

# 3) Start low-level solver
ros2 run mpc_low low_solver 

# 4) Start high-level solver
ros2 run mpc_high high_solver 

# 5) send command to start experiment 
ros2 topic pub --once /start std_msgs/msg/String data:\ \'\'\ 


ros2 run mpc_to high_solver


