#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import numpy as np
import torch
import time
import os

from numba import njit
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from mppi_controller.mppi import MPPIPlanner
from mppi_controller.mppi import MPPIConfig
from mppi_controller.c_func_w_vels import casadi_f0

print_counter = 0
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
data_indexes = [0, 1, 2, 3, 4, 5] 
position_reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

import itertools

sim_iterations = 10
experiment_length = 60.0  

human_data = torch.tensor([[0.30,   0.5,   0.45,   0.1000],  
          [0.0658,   0.4526,   0.8624,   0.2500],
          [0.0844,   0.7044,   0.9207,   0.1500],
          [0.2083,   0.3075,   1.0208,   0.1500],
          [0.0556,   0.6289,   0.7595,   0.1500],
          [0.2024,   0.2732,   0.8478,   0.1500],
          [0.0267,   0.5535,   0.5983,   0.1500],
          [0.1965,   0.2389,   0.6749,   0.1500],
         [-0.0208,   0.3964,   0.5857,   0.1000],
          [0.0546,   0.2951,   0.6132,   0.1000],
         [-0.1062,   0.2444,   0.5897,   0.1300],
         [-0.0998,   0.3062,   0.5387,   0.1300],
          [0.1908,   0.5290,   1.0016,   0.2000],
          [0.2106,   0.4602,   0.6915,   0.2500]], device='cuda')

@njit
def my_func(samples, cost_constr, iterations, human_spheres, p_final_cart):
    for i in range(iterations):
        response = casadi_f0(samples[i,0:6], samples[i,6:12], human_spheres, p_final_cart)
        cost_constr[i] = sum([(item*10000 if item>0.0 else 0.0) for item in response])
    return cost_constr

class MPPIController(Node):

    def __init__(self, data_array, perf_array, parameter_iterator, file_name=None):
        super().__init__('mppi_controller')
        self.file_name = file_name
            
        self.cmd_pub = self.create_publisher(Float64MultiArray, 
                                '/my_UR5/velocity_command', 1)
        self.js_sub = self.create_subscription(JointState, '/joint_states', 
                                        self.listener_callback, 1)
        self.subscription_human = self.create_subscription( Float64MultiArray, 
                                '/human_pose', self.human_data_callback, 1)

        self.data_array = data_array
        self.perf_array = perf_array
        self.task_counter = 0

        timer_period = 0.050
        self._dt = timer_period
        self.read_goal = [[0.0, -2.3, -1.1, -1.2, -1.2, 0.5],
            [3.0, -1.6, -1.7, -1.7, -1.7, 1.0]]
        self.currently_selected_goal = 0
        self.target = np.array(self.read_goal[self.currently_selected_goal])
        self.goal = torch.tensor(self.target, device='cuda')

        self.human_data = human_data.clone()

        # set mppi configs and create mppi planner
        self.experiment_duration = experiment_length 

        self.samples = 3000# 3250
        self.horizon = 10
        self.u_range = 1.2
        self.sigma = 0.1
        self.goal_pose_weight = 1.0

        self.action_vels_weight = 1.0 #25.0
        self.parameter_iterator = 0
        self._dt = 0.050 # 50ms
        ACTION_LOW = -1.2
        ACTION_HIGH = 1.2
        sigma = 1.0
        nu = 6
        
        cfg = MPPIConfig()
        cfg.device = "cuda:0"
        cfg.mppi_mode = 'halton-spline'
        cfg.sampling_method = "random"
        cfg.num_samples = 3000
        cfg.horizon = 10
        cfg.lambda_ = 0.1
        cfg.u_min = [ACTION_LOW, ACTION_LOW, ACTION_LOW, ACTION_LOW, ACTION_LOW, ACTION_LOW]
        cfg.u_max = [ACTION_HIGH, ACTION_HIGH, ACTION_HIGH, ACTION_HIGH, ACTION_HIGH, ACTION_HIGH]
        cfg.u_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cfg.noise_sigma = [list(x) for x in np.diag([sigma]*nu)]
        cfg.update_cov = False
        cfg.rollout_var_discount = 0.95
        cfg.sample_null_action = False
        cfg.noise_abs_cost = False
        cfg.filter_u = True
        cfg.use_priors = False
        
        
        self.planner = MPPIPlanner(
            cfg=cfg,
            nx=12,
            dynamics=self.dynamics,
            running_cost=self.running_cost,
        )
        
        print('Starting_experiment')
        self.experiment_start = time.time()
        right_now = self.get_clock().now()
        self.perf_array.append([right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1], self.task_counter])
        self.timer = self.create_timer(timer_period, self.node_callback)

    def listener_callback(self, msg):
        global print_counter, position_reading
        name_array = msg.name
        data_indexes = [0,1,2,3,4,5]
        if len(name_array)<6:
            print(name_array)
            return
        for j in range(len(joint_names)):
            index = name_array.index(joint_names[j])
            data_indexes[j] = index
        print_counter = print_counter + 1
        positions = [msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, msg.position[data_indexes[0]], msg.position[data_indexes[1]], msg.position[data_indexes[2]],
        msg.position[data_indexes[3]], msg.position[data_indexes[4]], msg.position[data_indexes[5]],
        msg.velocity[data_indexes[0]], msg.velocity[data_indexes[1]], msg.velocity[data_indexes[2]],
        msg.velocity[data_indexes[3]], msg.velocity[data_indexes[4]], msg.velocity[data_indexes[5]]]

        position_reading = np.array(positions[1:13])

    def node_callback(self):
        global position_reading
        # step the environment
        j_error=max(abs(self.target-position_reading[0:6]))
        if (j_error<0.05): 
            self.currently_selected_goal = (self.currently_selected_goal + 1) % 2;
            self.target = np.array(self.read_goal[self.currently_selected_goal])
            self.goal = torch.tensor(self.target, device='cuda')
            if self.currently_selected_goal == 0:
                right_now = self.get_clock().now()
                self.task_counter = self.task_counter + 1
                self.perf_array.append([right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1], self.task_counter])
        
        observation = np.array([position_reading[0], position_reading[1], position_reading[2], 
                       position_reading[3], position_reading[4], position_reading[5], 
                       position_reading[6], position_reading[7], position_reading[8], 
                       position_reading[9], position_reading[10], position_reading[11]]).T
        print("obs", observation)
        # calculate action
        start = time.time()
        new_action = self.planner.command(observation)
        mppi_action = new_action.cpu().numpy()
        # send command to robot
        msg = Float64MultiArray()
        msg.data = [float(x) for x in mppi_action]
        print("acti", [float(x) for x in mppi_action])
        self.cmd_pub.publish(msg)
        stop = time.time()
        #print(position_reading[0:7])
        # state recorder
        right_now = self.get_clock().now()
        line = [right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1], stop-start, j_error, 
                self.target[0], self.target[1], self.target[2], self.target[3], self.target[4], self.target[5], 
                position_reading[0], position_reading[1], position_reading[2], position_reading[3], position_reading[4], position_reading[5]]

        # --- append human spheres to CSV row (14 x 4 values -> 56 floats) ---
        # Ensure human_data is available and move to CPU if on GPU
        try:
            hd = self.human_data
            if isinstance(hd, torch.Tensor):
                hd_cpu = hd.detach().cpu().numpy().reshape(-1)  # flatten
            else:
                hd_cpu = np.array(hd).reshape(-1)
            # extend line with human sphere values as floats
            human_row = [float(x) for x in hd_cpu.tolist()]
        except Exception as e:
            # if human data not available for some reason, fill with NaNs to keep row length constant
            human_row = [float('nan')] * (14 * 4)
            self.get_logger().warning(f"human_data unavailable at logging step: {e}")

        line.extend(human_row)

        # append to internal buffer
        self.data_array.append(line)
        #print(self.target-position_reading[0:7])
        #print('time:',round(stop-start,3),'error:',round(j_error,3),'generated_command:', mppi_action)
        elapsed_time = time.time()-self.experiment_start
        #print('Elapsed_time:', elapsed_time, end='\r')
        print("goal",self.target)
        print("pose",position_reading[0:6])
        if (elapsed_time>self.experiment_duration): 
            tasks_duration = [x[0]+x[1]*1e-9 for x in self.perf_array]
            tasks_duration = np.array(tasks_duration[1:]) - np.array(tasks_duration[:-1]) 
            if len(tasks_duration)>2:
                tasks_duration = tasks_duration[1:]
                print('Finished_tasks:', self.perf_array[-1][2], max(tasks_duration), min(tasks_duration), sum(tasks_duration)/len(tasks_duration))
                print(self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight)
                record = [self.perf_array[-1][2], max(tasks_duration), min(tasks_duration), sum(tasks_duration)/len(tasks_duration), self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight]
                if self.file_name is not None:
                    self.file_name.write(" ".join([str(x) for x in record]))
                    self.file_name.write("\n")
                    self.file_name.flush()
                    os.fsync(self.file_name.fileno())
                #print(param_combinations[self.parameter_iterator+1])
            else:
                 record = [self.perf_array[-1][2], j_error, 0, 0, self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight]
                 if self.file_name is not None:
                    self.file_name.write(" ".join([str(x) for x in record]))
                    self.file_name.write("\n")
                    self.file_name.flush()
                    os.fsync(self.file_name.fileno())
            #print(tasks_duration)
            print('*\n**\n')
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.cmd_pub.publish(msg)
            raise SystemExit
    

    def dynamics(self, state, perturbed_action, t=None):
        theta = state[:,0:6]
        theta_dot = perturbed_action

        new_theta = theta + theta_dot * self._dt
        new_state = torch.cat((new_theta, theta_dot), dim=1)
        return new_state, perturbed_action

    def human_data_callback(self, msg):
        temp = np.array(msg.data)
        self.human_data = torch.tensor(
            temp.reshape((14, 4)), 
            device='cuda'
        )

    def running_cost(self, state, action=None):
        pose = state[:,0:6]
        vels = action if action is not None else state[:, 6:12]
        goal = self.goal

        human_spheres = self.human_data.cpu().numpy().flatten()

        weights = torch.tensor([100.0, 100.0, 70.0, 70.0, 50.0, 50.0], device='cuda').float()
        cost_state = torch.sum(weights*torch.square(goal-pose), dim=1)
        cost_action = 10.0 * torch.sum(torch.square(vels), dim=1)
        state_array = state[:,0:12].cpu().numpy()
        cost_constr = np.zeros((len(state_array),1))
        iterations = len(state_array)
        p_final_cart = [0.0, 0.0, 0.0]
        cost_constr = my_func(state_array, cost_constr, iterations, human_spheres, p_final_cart)
        cost_constr = torch.tensor(cost_constr, device = 'cuda')
        cost_constr = torch.squeeze(cost_constr)
        print(cost_state, cost_action, cost_constr)
        cost = cost_state + cost_action + cost_constr
        return cost
    

def main(args=None):
    global position_reading, human_data
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
            np.savetxt("mppi_stats"+str(i)+".csv", np.array(data_array), delimiter=',')
            np.savetxt("mppi_perf"+str(i)+".csv", np.array(perf_array), delimiter=',')
            minimal_subscriber.destroy_node()
            pass
    

    f.close()
    print('Experiment_finished')
    

if __name__ == '__main__':
    main()
