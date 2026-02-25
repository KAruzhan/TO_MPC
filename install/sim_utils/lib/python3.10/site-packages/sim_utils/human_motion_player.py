#!/usr/bin/env python3
import rclpy
import time
import numpy as np
import os

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
import math
import json

from numpy import genfromtxt
from ament_index_python.packages import get_package_share_directory  

file_data = genfromtxt(
    os.path.join(
        get_package_share_directory('sim_utils'),  # Get package share directory
        'data',
        'Participant_541.csv' # 'human_motions_1.csv' #
    ),
    skip_header=1,
    delimiter=',',
)

# auxilaru functions for coordanate transformations (for example from frame of camera to frame of robot)
def rotate_by_matrix(p1, matrix):
    p1_rotated = np.dot(np.array(matrix),(np.array(p1).T))
    return p1_rotated

def rotate_to_robot(row):
    rotation_matrix = [
        [-0.99863968, -0.0107969,  -0.05101196,  0.68227144],
        [-0.05096973, -0.00415132,  0.99869157,  0.04468653],
        [-0.01099454,  0.99993309,  0.00359536, -0.99938648],
        [0.0, 0.0, 0.0, 1.0]
    ]  
    for i in range(0, 14):
        p1 = [row[i*3+0], row[i*3+1], row[i*3+2], 1] 
        rotation_result = self.rotate_by_matrix(p1, rotation_matrix)
        row[i*3+0] = rotation_result[0]
        row[i*3+1] = rotation_result[1]
        row[i*3+2] = rotation_result[2]
    return row

class SphereMover(Node):

    def __init__(self):   # here was a typo 'def __init(self)'
        super().__init__('sphere_mover')
        self.start_time = 0.0
        #is_success, msg = wait_for_message(Clock, self, '/clock', time_to_wait=10.0)
        #self.clock_subscription = self.create_subscription(Clock,'/clock',self.clock_callback,1)
        self.start_time = self.get_clock().now().nanoseconds*1.0e-9
        #if is_success:
        #    self.start_time = msg.clock.sec + msg.clock.nanosec*1e-9
        self.get_logger().info(f"Simulation Start Time: {self.start_time:.10f}")
        self.sim_time = 0.0
        self.my_data = [0.0] * 42
        self.iterator = 0
        self.plot_iterator = 0
        self.data_len = len(file_data)
        self.sphere_data = [0.0] * 56
        self.time_arr = file_data[:,-1]
        self.gazebo = False
        
        is_success, msg = wait_for_message(String, self, '/start')
        if is_success:
            self.start_time = self.get_clock().now().nanoseconds*1.0e-9
        
        timer_period = 1.0/120.0  # seconds (data in .csv file recorded at 120Hz)
        self.timer = self.create_timer(timer_period, self.pose_publisher)
        
        
        
        #sphere_radi = [0.171+0.05, 0.311, 0.161, 0.161, 0.131, 0.131, 0.131, 0.131, 0.111, 0.111, 0.121, 0.121, 0.171, 0.151]; # unchanged
        #self.sphere_radi = [0.511, 0.601, 0.451, 0.451, 0.421, 0.421, 0.421, 0.421, 0.401, 0.401, 0.411, 0.411, 0.461, 0.441] # d_bar 0.29
        self.sphere_radi = [0.351, 0.441, 0.291, 0.291, 0.261, 0.261, 0.261, 0.261, 0.241, 0.241, 0.251, 0.251, 0.301, 0.281] # d_bar 0.130
        self.pub_plotter = self.create_publisher(Float64MultiArray, '/Sphere_poses_plotter', 1) # only latest published data is needed
        self.pub_spheres = self.create_publisher(Float64MultiArray,'/human_pose', 1)
        if self.gazebo:
            self.gazebo_sphere1 = self.create_publisher(Pose,'/sphere1', 1)
            self.gazebo_sphere2 = self.create_publisher(Pose,'/sphere2', 1)
            self.gazebo_sphere3 = self.create_publisher(Pose,'/sphere3', 1)
            self.gazebo_sphere4 = self.create_publisher(Pose,'/sphere4', 1)
            self.gazebo_sphere5 = self.create_publisher(Pose,'/sphere5', 1)
            self.gazebo_sphere6 = self.create_publisher(Pose,'/sphere6', 1)
            self.gazebo_sphere7 = self.create_publisher(Pose,'/sphere7', 1)
            self.gazebo_sphere8 = self.create_publisher(Pose,'/sphere8', 1)
            self.gazebo_sphere9 = self.create_publisher(Pose,'/sphere9', 1)
            self.gazebo_sphere10 = self.create_publisher(Pose,'/sphere10', 1)
            self.gazebo_sphere11 = self.create_publisher(Pose,'/sphere11', 1)
            self.gazebo_sphere12 = self.create_publisher(Pose,'/sphere12', 1)
            self.gazebo_sphere13 = self.create_publisher(Pose,'/sphere13', 1)
            self.gazebo_sphere14 = self.create_publisher(Pose,'/sphere14', 1)
            self.gazebo_sender_arr = [self.gazebo_sphere1, self.gazebo_sphere2, self.gazebo_sphere3, self.gazebo_sphere4, self.gazebo_sphere5, self.gazebo_sphere6, self.gazebo_sphere7,
                self.gazebo_sphere8, self.gazebo_sphere9, self.gazebo_sphere10, self.gazebo_sphere11, self.gazebo_sphere12, self.gazebo_sphere13, self.gazebo_sphere14]
        #self.subscription = self.create_subscription(String, 'simulation_status', self.callback, 10)
        
    #def callback(self, msg):
    #    if msg.data == "Start":
    #        self.start_time= self.get_clock().now().nanoseconds / 1.0e9

    def pose_publisher(self):
        # no need to use while loop, we created timer object in initialization of the node
        try: # add try exepc to finish node gracefully
            if self.start_time != 0.0:
                self.sim_time = self.get_clock().now().nanoseconds*1.0e-9 - self.start_time
                current_point = self.sim_time % file_data[-1][-1]
                self.iterator = np.argmin(abs(self.time_arr - current_point))
                if (file_data[self.iterator][-1] - current_point)> 0 and self.iterator> 0:
                    self.iterator = self.iterator - 1
                self.my_data = file_data[self.iterator].copy()

                for j in range(14):
                    #self.my_data[j * 3 + 0] = (self.my_data[j * 3 + 0] + 1.00) #- 0.5
                    #self.my_data[j * 3 + 1] = (self.my_data[j * 3 + 1] + 0.50)
                    #self.my_data[j * 3 + 2] = (self.my_data[j * 3 + 2] - 1.2)
                    self.my_data[j * 3 + 0] = -(self.my_data[j * 3 + 0]+1.0) #- 0.5
                    self.my_data[j * 3 + 1] = -(self.my_data[j * 3 + 1]+0.5)
                    self.my_data[j * 3 + 2] = (self.my_data[j * 3 + 2]-1.2)
                    self.sphere_data[j * 4 + 0] = self.my_data[j * 3 + 0]
                    self.sphere_data[j * 4 + 1] = self.my_data[j * 3 + 1]
                    self.sphere_data[j * 4 + 2] = self.my_data[j * 3 + 2]
                    self.sphere_data[j * 4 + 3] = self.sphere_radi[j]

                # Publish sphere data for simulation
                sphere_data_msg = Float64MultiArray()
                sphere_data_msg.data = self.sphere_data[0:56]
                self.pub_spheres.publish(sphere_data_msg)
            
                self.plot_iterator = self.plot_iterator + 1
                if self.plot_iterator % 12 == 0:
                    # Publish sphere data for plotting
                    sphere_state_msg = Float64MultiArray()
                    sphere_state_msg.data = [float(x) for x in self.my_data[0:42]]
                    self.pub_plotter.publish(sphere_state_msg)
                
                if self.plot_iterator % 12 == 0 and self.gazebo:
                    #self.get_logger().info(f"Simulation Time: {self.sim_time:.10f}")  # Log simulation time
                    for i in range(14):
                        # send coordinates for each sphere
                        sphere_coordinates = Pose()
                        sphere_coordinates.position.x = self.sphere_data[i*4+0]
                        sphere_coordinates.position.y = self.sphere_data[i*4+1]
                        sphere_coordinates.position.z = self.sphere_data[i*4+2]
                        self.gazebo_sender_arr[i].publish(sphere_coordinates)
                
                if self.plot_iterator % 120 == 0:
                    self.get_logger().info(f"Simulation Time: {self.sim_time:.10f}")  # Log simulation time
        except KeyboardInterrupt:
            print('Stopped')
            #break
            raise SystemExit           # <--- here is we exit the node

    # function to measure time from external clock (for gazebo)
    def clock_callback(self, data):
        self.sim_time = data.clock.sec + data.clock.nanosec*1e-9

def main(args=None):
    rclpy.init(args=args)
    
    sphere_publisher_node = SphereMover()
    
    rclpy.spin(sphere_publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sphere_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
