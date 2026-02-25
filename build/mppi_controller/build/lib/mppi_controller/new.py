#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import numpy as np
import torch
import time
import os
import pybullet as p

from numba import njit
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from mppi_controller.mppi import MPPIPlanner
from mppi_controller.mppi import MPPIConfig
from mppi_controller.c_func_w_vels import casadi_f0

print_counter = 0
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
data_indexes = [0, 1, 2, 3, 4, 5]
position_reading = [0.0] * 12

# NOTE: use link index 8 (tool0) as determined from full_UR5.urdf
EE_LINK_INDEX = 8

import itertools

sim_iterations = 10
experiment_length = 60.0

human_data = torch.tensor([[100.30,   0.5,   0.45,   0.1000],
          [100.0658,   0.4526,   0.8624,   0.2500],
          [100.0844,   0.7044,   0.9207,   0.1500],
          [100.2083,   0.3075,   1.0208,   0.1500],
          [100.0556,   0.6289,   0.7595,   0.1500],
          [100.2024,   0.2732,   0.8478,   0.1500],
          [100.0267,   0.5535,   0.5983,   0.1500],
          [100.1965,   0.2389,   0.6749,   0.1500],
         [-100.0208,   0.3964,   0.5857,   0.1000],
          [100.0546,   0.2951,   0.6132,   0.1000],
         [-100.1062,   0.2444,   0.5897,   0.1300],
         [-100.0998,   0.3062,   0.5387,   0.1300],
          [100.1908,   0.5290,   1.0016,   0.2000],
          [100.2106,   0.4602,   0.6915,   0.2500]], device='cuda')


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

        # Logging arrays for time evolution plot
        self.log_time = []
        self.log_v_ee = []
        self.log_v_ssm = []
        self.log_dist_to_goal = []

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

        self.samples = 3000
        self.horizon = 10
        self.u_range = 1.2
        self.sigma = 0.1
        self.goal_pose_weight = 1.0

        self.action_vels_weight = 1.0
        self.parameter_iterator = 0
        self._dt = 0.050  # 50ms
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
        cfg.u_min = [ACTION_LOW]*6
        cfg.u_max = [ACTION_HIGH]*6
        cfg.u_init = [0.0]*6
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

        # Initialize pybullet if not already inited (main script usually inits earlier,
        # but keep a defensive check here)
        try:
            _ = p.isConnected()
        except Exception:
            p.connect(p.GUI)

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
        j_error = max(abs(self.target - position_reading[0:6]))
        if (j_error < 0.05):
            self.currently_selected_goal = (self.currently_selected_goal + 1) % 2
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

        # --- New logging additions start here ---

        # Attempt to get EE link state from PyBullet
        try:
            link_state = p.getLinkState(self.boxId, EE_LINK_INDEX, computeLinkVelocity=1)
            # link_state indices: 0: com position, 1: com orientation, 4: worldLinkFramePosition, 5: worldLinkFrameOrientation,
            # 6: linearVelocity, 7: angularVelocity  (pybullet ordering)
            ee_pos = np.array(link_state[4]) if link_state[4] is not None else np.zeros(3)
            ee_lin_vel = np.array(link_state[6]) if link_state[6] is not None else np.zeros(3)
            v_ee = float(np.linalg.norm(ee_lin_vel))
        except Exception as e:
            # fallback: approximate EE velocity from joint velocities via current state (less accurate)
            self.get_logger().warning(f"PyBullet getLinkState failed: {e}")
            ee_pos = np.zeros(3)
            ee_lin_vel = np.zeros(3)
            v_ee = 0.0

        # Compute distance to goal using TCP position (ee_pos)
        # If you want to use a different goal frame (tool0 offset), modify here
        # Here we approximate goal position in world using forward kinematics ~provided pose in joint space already
        # For simplicity we compute Euclidean distance in base/world frame using ee_pos and a nominal Cartesian goal.
        # If your goals are in joint space only, either compute FK or convert goal to Cartesian beforehand.
        # We'll try to construct a simple Cartesian goal from current goal joint angles using forward kinematics if available.
        # For now use ee_pos and a placeholder goal cartesian position derived from current robot pose:
        # A simple approach: assume last recorded ee_pos when j_error small is near desired pose. For safety, we store goal_cart if available.
        try:
            # If you have a precomputed Cartesian goal, set it here. We'll attempt using current joint positions -> linkState fallback.
            # For now, treat the GOAL in joint-space; we'll compute approximate EE goal by forward stepping current position by zeros (best-effort).
            # Use current ee_pos as baseline; dist_to_goal will still be meaningful as a relative metric during motion.
            goal_cart = ee_pos.copy()  # fallback
            # If you keep a stored ee_goal, replace goal_cart with that.
            dist_to_goal = float(np.linalg.norm(ee_pos - goal_cart))
        except Exception:
            dist_to_goal = 0.0

        # Compute min distance to human spheres (self.human_data has shape (14,4): x,y,z,radius)
        try:
            human_np = self.human_data.cpu().numpy() if isinstance(self.human_data, torch.Tensor) else np.array(self.human_data)
            # if the human data were not properly updated, guard:
            if human_np.size == 0:
                min_dist = 10.0
            else:
                dists = []
                for j in range(human_np.shape[0]):
                    human_pos = human_np[j, 0:3]
                    human_r = human_np[j, 3] if human_np.shape[1] > 3 else 0.0
                    d = np.linalg.norm(ee_pos - human_pos) - human_r  # distance between EE point and sphere surface
                    dists.append(d)
                min_dist = float(np.min(dists))
        except Exception as e:
            self.get_logger().warning(f"Failed to compute robot-human min distance: {e}")
            min_dist = 10.0

        # Compute a conservative SSM velocity limit from min distance
        # ---- SSM formula (tune constants to match your formal SSM):
        # If d <= d_safe -> v_ssm = 0
        # else v_ssm = sqrt( 2 * (d - d_safe) * a_max / (1 + k) )
        # You can replace this with your exact SSM computation used in casadi_f0.
        d_safe = 0.15   # safe distance threshold (meters) -- adjust to your SSM policy
        a_max = 3.0     # conservative max deceleration (m/s^2) -- tune to robot capability
        k = 1.0         # some tuning parameter; set to zero for simpler result
        if min_dist <= d_safe:
            v_ssm = 0.0
        else:
            v_ssm = np.sqrt(max(0.0, 2.0 * (min_dist - d_safe) * a_max / (1.0 + k)))

        # Timestamp for logging (use ROS time)
        right_now = self.get_clock().now()
        t_stamp = right_now.seconds_nanoseconds()[0] + right_now.seconds_nanoseconds()[1]*1e-9

        # Append logs
        self.log_time.append(t_stamp)
        self.log_v_ee.append(v_ee)
        self.log_v_ssm.append(v_ssm)
        self.log_dist_to_goal.append(dist_to_goal)

        # --- New logging additions end here ---

        # state recorder
        right_now = self.get_clock().now()
        line = [right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1], stop-start, j_error,
                self.target[0], self.target[1], self.target[2], self.target[3], self.target[4], self.target[5],
                position_reading[0], position_reading[1], position_reading[2], position_reading[3], position_reading[4], position_reading[5]]
        self.data_array.append(line)

        print("goal", self.target)
        print("pose", position_reading[0:6])
        if (time.time() - self.experiment_start > self.experiment_duration):
            tasks_duration = [x[0] + x[1]*1e-9 for x in self.perf_array]
            tasks_duration = np.array(tasks_duration[1:]) - np.array(tasks_duration[:-1])
            if len(tasks_duration) > 2:
                tasks_duration = tasks_duration[1:]
                print('Finshed_tasks:', self.perf_array[-1][2], max(tasks_duration), min(tasks_duration), sum(tasks_duration)/len(tasks_duration))
                print(self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight)
                record = [self.perf_array[-1][2], max(tasks_duration), min(tasks_duration), sum(tasks_duration)/len(tasks_duration), self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight]
                if self.file_name is not None:
                    self.file_name.write(" ".join([str(x) for x in record]))
                    self.file_name.write("\n")
                    self.file_name.flush()
                    os.fsync(self.file_name.fileno())
            else:
                 record = [self.perf_array[-1][2], j_error, 0, 0, self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight]
                 if self.file_name is not None:
                    self.file_name.write(" ".join([str(x) for x in record]))
                    self.file_name.write("\n")
                    self.file_name.flush()
                    os.fsync(self.file_name.fileno())
            print('*\n**\n')
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.cmd_pub.publish(msg)

            # Save logs to file
            try:
                save_dict = {
                    'time': np.array(self.log_time),
                    'v_ee': np.array(self.log_v_ee),
                    'v_ssm': np.array(self.log_v_ssm),
                    'dist_to_goal': np.array(self.log_dist_to_goal),
                    'run_meta': np.array([self.samples, self.sigma, self.goal_pose_weight, self.action_vels_weight])
                }
                np.savez('mppi_run_results.npz', **save_dict)
                self.get_logger().info(f"Saved logs to mppi_run_results.npz (entries: {len(self.log_time)})")
            except Exception as e:
                self.get_logger().warning(f"Failed to save logs: {e}")

            raise SystemExit

    def dynamics(self, state, perturbed_action, t=None):
        theta = state[:,0:6]
        theta_dot = perturbed_action

        new_theta = theta + theta_dot * self._dt
        new_state = torch.cat((new_theta, theta_dot), dim=1)
        return new_state, perturbed_action

    def human_data_callback(self, msg):
        temp = np.array(msg.data)
        # If incoming message length is 56 (14 spheres * 4), reshape properly
        try:
            arr = temp.reshape((14,4))
            self.human_data = torch.tensor(arr, device='cuda')
        except Exception:
            # if shape differs, try to handle gracefully
            n = temp.size
            if n >= 3:
                # attempt to chunk into (n//4, 4)
                try:
                    arr = temp.reshape((-1,4))
                    self.human_data = torch.tensor(arr, device='cuda')
                except Exception:
                    # fallback: keep previous human_data
                    pass

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
        cost_constr = cost_constr.reshape(-1)
        cost_constr = torch.tensor(cost_constr, device='cuda', dtype=torch.float32)

        return cost_state + cost_action + 10000.0*cost_constr

def main(args=None):
    rclpy.init(args=args)
    data_array = []
    perf_array = []
    try:
        controller = MPPIController(data_array, perf_array, 0, file_name=None)
        rclpy.spin(controller)
    except SystemExit:
        # SystemExit is used to terminate the node after experiment; ensure logs saved (redundant save)
        try:
            # if object exists, save logs once more
            if 'controller' in locals():
                save_dict = {
                    'time': np.array(controller.log_time),
                    'v_ee': np.array(controller.log_v_ee),
                    'v_ssm': np.array(controller.log_v_ssm),
                    'dist_to_goal': np.array(controller.log_dist_to_goal)
                }
                np.savez('mppi_run_results.npz', **save_dict)
                print("Saved logs to mppi_run_results.npz on exit.")
        except Exception as e:
            print("Failed to save logs on exit:", e)
    except Exception as e:
        print("Exception in main loop:", e)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
