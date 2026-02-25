#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import numpy as np
import torch
import time

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from mppi_controller.mppi import MPPIPlanner
from mppi_controller.mppi import MPPIConfig

print_counter = 0
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
data_indexes = [0, 1, 2, 3, 4, 5] 
position_reading = np.zeros(12)

max_vel = 0.5

sim_iterations = 1
experiment_length = 120.0  

class MPPIController(Node):
    def __init__(self, data_array, perf_array, parameter_iterator, file_name=None):
        super().__init__('mppi_controller')
        
        self.cmd_pub = self.create_publisher(Float64MultiArray, 
                                '/my_UR5/velocity_command', 1)
            
        self.js_sub = self.create_subscription(JointState, '/joint_states', 
                                        self.listener_callback, 1)

        self.received_js = False

        self.data_array = data_array
        self.perf_array = perf_array
        self.task_counter = 0
        timer_period = 0.008
        self._dt = timer_period
        self.read_goal = [[0.0, -2.3, -1.1, -1.2, -1.2, 0.5],
            [3.0, -1.6, -1.7, -1.7, -1.7, 1.0]]
        self.currently_selected_goal = 0
        self.target = np.array(self.read_goal[self.currently_selected_goal])
        self.goal = torch.tensor(self.target, device='cuda')
        # set mppi configs and create mppi planner
        self.experiment_duration = experiment_length 

        ACTION_LOW = -1.2
        ACTION_HIGH = 1.2
        sigma = 1.0
        nu = 6
        
        cfg = MPPIConfig()
        cfg.device = "cuda:0"
        cfg.mppi_mode = 'halton-spline'
        cfg.sampling_method = "random"
        cfg.num_samples = 20
        cfg.horizon = 10
        cfg.lambda_ = 0.1
        cfg.u_min = [ACTION_LOW, ACTION_LOW, ACTION_LOW, ACTION_LOW, ACTION_LOW, ACTION_LOW]
        cfg.u_max = [ACTION_HIGH, ACTION_HIGH, ACTION_HIGH, ACTION_HIGH, ACTION_HIGH, ACTION_HIGH]
        cfg.noise_sigma = [list(x) for x in np.diag([sigma]*nu)]
        cfg.update_cov = False
        cfg.rollout_var_discount = 0.95
        cfg.sample_null_action = False
        cfg.noise_abs_cost = False
        cfg.filter_u = True
        cfg.use_priors = False
        
        
        self.planner = MPPIPlanner(
            cfg=cfg,
            nx=6,
            dynamics=self.dynamics,
            running_cost=self.running_cost,
        )
        
        print('Starting_experiment')
        self.experiment_start = time.time()
        right_now = self.get_clock().now()
        self.perf_array.append([right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1], self.task_counter])
        self.timer = self.create_timer(timer_period, self.node_callback)

    def listener_callback(self, msg):
        global position_reading, print_counter

        if len(msg.name) < 6:
            return

        for j in range(6):
            data_indexes[j] = msg.name.index(joint_names[j])

        position_reading = np.array([
            msg.position[data_indexes[0]],
            msg.position[data_indexes[1]],
            msg.position[data_indexes[2]],
            msg.position[data_indexes[3]],
            msg.position[data_indexes[4]],
            msg.position[data_indexes[5]],
            msg.velocity[data_indexes[0]],
            msg.velocity[data_indexes[1]],
            msg.velocity[data_indexes[2]],
            msg.velocity[data_indexes[3]],
            msg.velocity[data_indexes[4]],
            msg.velocity[data_indexes[5]],
        ])

        self.received_js = True   
        print_counter += 1


    def node_callback(self):
        global position_reading, print_counter

        if not self.received_js:
            msg = Float64MultiArray()
            msg.data = [0.0]*6
            self.cmd_pub.publish(msg)
            return

        j_error = max(abs(self.target - position_reading[0:6]))

        if j_error < 0.05 and np.linalg.norm(position_reading[6:12]) < 0.05:  
            self.currently_selected_goal = (self.currently_selected_goal + 1) % 2
            self.target = np.array(self.read_goal[self.currently_selected_goal])
            self.goal = torch.tensor(self.target, device='cuda')

            if self.currently_selected_goal == 0:
                right_now = self.get_clock().now()
                self.task_counter += 1
                self.perf_array.append([
                    right_now.seconds_nanoseconds()[0],
                    right_now.seconds_nanoseconds()[1],
                    self.task_counter
                ])

        observation = np.array([position_reading[0], position_reading[1], position_reading[2], 
                       position_reading[3], position_reading[4], position_reading[5]]).T
        print("obs", observation)
        # calculate action
        start = time.time()
        new_action = self.planner.command(observation)
        mppi_action = new_action.cpu().numpy()
        mppi_action = np.clip(mppi_action, -max_vel, max_vel)
        # send command to robot
        msg = Float64MultiArray()
        msg.data = [float(x) for x in mppi_action]
        print("acti", [float(x) for x in mppi_action])
        self.cmd_pub.publish(msg)
        stop = time.time()

        if print_counter % 50 == 0:    
            print("Goal:", self.target)
            print("Pose:", position_reading[0:6])
            print("Cmd :", mppi_action)

        # state recorder
        right_now = self.get_clock().now()
        line = [right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1], stop-start, j_error, 
                self.target[0], self.target[1], self.target[2], self.target[3], self.target[4], self.target[5], 
                position_reading[0], position_reading[1], position_reading[2], position_reading[3], position_reading[4], position_reading[5]]
        self.data_array.append(line)
        
        if time.time() - self.experiment_start > self.experiment_duration:
            msg = Float64MultiArray()
            msg.data = [0.0]*6
            self.cmd_pub.publish(msg)
            raise SystemExit

    def dynamics(self, state, actions, t=None):
        theta = state[:, 0:6]
        new_theta = theta + actions * self._dt
        #print(state, actions)
        return new_theta, actions 

    def running_cost(self, state, action=None):
        theta = state[:,0:6]
        goal = self.goal 
        weights = torch.tensor([30.0, 30.0, 20.0, 20.0, 15.0, 15.0], device='cuda').float()
        cost_state = torch.sum(weights*torch.square(goal-theta), dim=1)
        if action:
            theta_dot = action
            cost_action = 300.0*torch.sum(torch.square(theta_dot), dim=1)
        else:
            cost_action = 0.0
        print("Cost", cost_state, cost_action)
        return cost_state + cost_action
    

def main(args=None):
    global position_reading
    rclpy.init(args=args)
    f = open("tune_test.txt", "w")


    print('Start recording...')
    for i in range(sim_iterations):
        try:
            data_array = []
            perf_array = []
            minimal_subscriber = MPPIController(data_array, perf_array, i, f)
            rclpy.spin(minimal_subscriber)

        except SystemExit: 
            print("recorded ",len(data_array),"samples")
            np.savetxt("mppi_robot_stats"+str(i)+".csv", np.array(data_array), delimiter=',')
            np.savetxt("mppi_robot_perf"+str(i)+".csv", np.array(perf_array), delimiter=',')
            minimal_subscriber.destroy_node()
            pass
    

    f.close()
    print('Experiment_finished')
    

if __name__ == '__main__':
    main()
