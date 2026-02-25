#!/usr/bin/env python3
import rclpy
import time
import numpy as np
import os

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
import math
import json

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
data_indexes = [0, 1, 2, 3, 4, 5] #[0, 4, 1, 5, 6, 2, 3]
q_dd_lim = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]


reference_position = [ 2.1635520695797757e-05,-1.5707770180168326, -2.4371119767526396e-05, -1.5707816583087153,-1.5051903083929119e-05,2.4819329780112913e-05]
kP = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

goal_joint = 5
step_state_array = [-1.0, 0.0, 1.0]
step_duration = 1.1
current_command_index = 0

class SphereMover(Node):

    def __init__(self, log_file):   # here was a typo 'def __init(self)'
        super().__init__('joint_tunner')
        self.start_time = self.get_clock().now().nanoseconds*1.0e-9
        self.current_goal_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_goal_position = reference_position
        self.current_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_subscription_state = self.create_subscription(JointState,'/joint_states',self.joint_callback,1)
        self.pub_comand = self.create_publisher(Float64MultiArray,'/ur_driver/joint_speed', 1)
        timer_period = 1.0/100.0  # seconds (data in .csv file recorded at 120Hz)
        self.timer = self.create_timer(timer_period, self.pose_publisher)
        self.start_step = time.time()
        self.log_file = log_file
        
        

    # function to measure time from external clock (for gazebo)
    def joint_callback(self, msg):
        name_array = msg.name
        for j in range(len(joint_names)):
            index = name_array.index(joint_names[j])
            data_indexes[j] = index
        self.current_velocity = [msg.velocity[data_indexes[0]], msg.velocity[data_indexes[1]], msg.velocity[data_indexes[2]],
        msg.velocity[data_indexes[3]], msg.velocity[data_indexes[4]], msg.velocity[data_indexes[5]]]
        self.current_position = [msg.position[data_indexes[0]], msg.position[data_indexes[1]], msg.position[data_indexes[2]],
        msg.position[data_indexes[3]], msg.position[data_indexes[4]], msg.position[data_indexes[5]]]

    def pose_publisher(self):
        global goal_joint, step_state_array, step_duration, current_command_index
        self.sim_time = self.get_clock().now().nanoseconds*1.0e-9 - self.start_time
        send_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(6):
            if (i==goal_joint):
                current_time = time.time()
                if (current_time-self.start_step)>step_duration:
                    self.start_step = current_time
                    current_command_index = (current_command_index + 1)%3
                send_velocity[i] = step_state_array[current_command_index]
            else:
                diff = self.current_goal_position[i]-self.current_position[i];
                send_velocity[i] = kP[i] * diff;
        logger_line = [current_time]
        logger_line.extend(self.current_velocity)
        logger_line.extend(send_velocity)
        logger_line.extend(self.current_position)
        self.log_file.write(" ".join([str(x) for x in logger_line]))
        self.log_file.write("\n")
        command_data_msg = Float64MultiArray()
        command_data_msg.data = send_velocity
        self.pub_comand.publish(command_data_msg)
        #self.get_logger().info(f"Simulation Time: {a_input:.5f}")

def main(args=None):
    rclpy.init(args=args)
    f = open("joint6_1.0.txt", "w")

    sphere_publisher_node = SphereMover(f)
    
    rclpy.spin(sphere_publisher_node)
    f.close()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sphere_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
