#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "acados_solver_holder.hpp"
#include "rclcpp/wait_for_message.hpp"

//#include "kinematics_functions.cpp"

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::ofstream MyFile;
std::ofstream MyPerformanceFile;

// float dist_v(Eigen::Vector3f v, Eigen::Vector3f w){
//     return (v-w).norm();
// }

// double get_min_dist_arr(double joint_position[6], double human_sphere[56], double min_dist[7], double spheres_dist[7], double end_effector_pose[3])
// {
//     double local_val = 10000;
//     double smallest_dist = 10000;
//     int number_of_spheres_robot = 7;
//     int number_of_spheres_human = 14;
//     double robot_spheres[7] = {0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.10};

//     Eigen::MatrixXf mat2 = get_cpose(joint_position[0], joint_position[1], 
//         joint_position[2], joint_position[3], joint_position[4], 
//         joint_position[5]);

//     for (int j = 0; j < number_of_spheres_robot; j++) {
//         Eigen::Vector3f w;
//         w = mat2.col(j).transpose();
//         for (int k = 0; k < number_of_spheres_human; k++) 
//         {
//             Eigen::Vector3f p(human_sphere[k*4+0],human_sphere[k*4+1],human_sphere[k*4+2]);
//             local_val = dist_v(w, p) - robot_spheres[j] - human_sphere[k*4+3];
//             if (local_val < min_dist[j]) {
//                 min_dist[j] = local_val;
//                 spheres_dist[j] = robot_spheres[j] + human_sphere[k*4+3];
//             }
//             //std::cout << k << ":" << p << std::endl;
//         }
//         if (smallest_dist > min_dist[j]) smallest_dist = min_dist[j];
//     }
//     Eigen::Vector3f end_effector_pose_vect = mat2.col(number_of_spheres_robot-1);

//     end_effector_pose[0] = end_effector_pose_vect.x();
//     end_effector_pose[1] = end_effector_pose_vect.y();
//     end_effector_pose[2] = end_effector_pose_vect.z();
//     return smallest_dist;
// }


// double get_spheres_velocity(double joint_position[6], double joint_velocities[6], double max_vell[7])
// {
//     Eigen::MatrixXf vell_mat = get_velocity(
//         joint_position[0], joint_position[1], joint_position[2], 
//         joint_position[3], joint_position[4], joint_position[5],
//         joint_velocities[0], joint_velocities[1], joint_velocities[2],
//         joint_velocities[3], joint_velocities[4], joint_velocities[5]);
//     double max_linear_vell = 0;
//     double temp_linear_vell = 0;
//     for (int k=0; k<7; k++) {
//         temp_linear_vell = sqrt(
//             vell_mat.coeff(k*3 + 0,0)*vell_mat.coeff(k*3 + 0,0) + 
//             vell_mat.coeff(k*3 + 1,0)*vell_mat.coeff(k*3 + 1,0) + 
//             vell_mat.coeff(k*3 + 2,0)*vell_mat.coeff(k*3 + 2,0)
//             );
//         max_vell[k] = temp_linear_vell;
//         if (max_linear_vell < temp_linear_vell) max_linear_vell = temp_linear_vell;
//     }
//     return max_linear_vell;
// }


// double rescale_velocities(double min_dist[7], double spheres_dist[7], double max_vell[7], double joint_vels[6])
// {
//     //*********************** Apply control ********************************
//     double lin_vell_limit_arr[7] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
//     double lin_vell_scale = 10.0;
//     double alpha = 0.75; //v_mpc_lim = sqrt(alpha^2.*((min_dist_arr+spheres_dist).^2-spheres_dist.^2));
//     double sqrt_temp_value = 0.00;

//     for (int i=0;i<7;i++) {
//       sqrt_temp_value = (min_dist[i]+spheres_dist[i])*(min_dist[i]+spheres_dist[i])-spheres_dist[i]*spheres_dist[i];
//       if (sqrt_temp_value<0) lin_vell_limit_arr[i] = 0.00000000000000;
//       else lin_vell_limit_arr[i] = alpha*sqrt(sqrt_temp_value);

//       if (lin_vell_limit_arr[i]<0) lin_vell_limit_arr[i] = 0.00000000000000;
//       double temp_scale = (lin_vell_limit_arr[i]/max_vell[i]);
//       if (lin_vell_scale>temp_scale) lin_vell_scale = temp_scale;
//     }
      
//     if (lin_vell_scale<1.0) {
//         for (int i = 0; i < 6; i++) joint_vels[i] = joint_vels[i]*lin_vell_scale;
//     }
//     return lin_vell_scale;
// }


class RobotCommandPublisher : public rclcpp::Node
{
public:
    RobotCommandPublisher() : Node("robot_command_publisher"), count_(0)
    {
        trajectory_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mpc_high_positions", 1);
        direct_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/my_UR5/velocity_command", 1); //run without low solver

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&RobotCommandPublisher::topic_callback, this, _1));
        
        // subscription_human_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        // "/human_pose", 1, std::bind(&RobotCommandPublisher::topic_human_callback, this, _1));

        
        
        MyFile.open("solver_stats_to_high1.csv"); //, std::ofstream::out | std::ofstream::app);
        MyPerformanceFile.open("solver_perf_to_high1.csv");

        nmpc_solver = std::make_shared<my_NMPC_solver>(1, number_of_current_steps); // Number of SQP steps       

        // Created timer with 500ms
        // 1) we send as a goal fixed position
        // format for sending the goal [ n_steps goal_1(6) time_1 goal_2(6) ... goal_n(6) ]
        // solution_msg.data[0] = steps
        // solution_msg.data[DOF + 1] where DOF = 7
        /*auto message = std_msgs::msg::Float64MultiArray();
        message.data.clear();
        int n_steps = 10;
        message.data.push_back(n_steps); // n_steps
        for (int j=0; j<n_steps;++j)
        {
            for (int i=0; i<7;++i) message.data.push_back(read_goal[0][i]);
            message.data.push_back(0.010);
        }
        trajectory_publisher_->publish(message);*/
    }
    
    void wait_for_start()
    {
        auto scan_message = std_msgs::msg::String();
        bool scan_found = false;

        while (rclcpp::ok() && !scan_found) {
            scan_found = rclcpp::wait_for_message(
                scan_message,
                shared_from_this(),  // This is now safe!
                "/start"   );
        }

        RCLCPP_INFO(this->get_logger(), "Message found");
        if (1)
        {
            // Get current time in seconds
            start_time = this->get_clock()->now();
            wall_start_time = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "Start received. Time: %f seconds", start_time.seconds());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Did not receive /start message in time.");
        }   
        
        timer_ = this->create_wall_timer
        (
            250ms,
            std::bind(&RobotCommandPublisher::timer_callback, this)
        );
    }

private:
    double experiment_duration = 60.0; //1816.29167;
    const bool bypass_low_solver = false;
    int number_of_current_steps = 10;
    double current_position[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double current_velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double goal_position[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double goal_queue[120] = {0.0};
    int currently_selected_goal = 0;
    // double human_sphere[56]= {10.0517,   0.5220,   1.0895,   0.1500,
    //               10.0658,   0.4526,   0.8624,   0.2500,
    //               10.0844,   0.7044,   0.9207,   0.1500,
    //               10.2083,   0.3075,   1.0208,   0.1500,
    //               10.0556,   0.6289,   0.7595,   0.1500,
    //               10.2024,   0.2732,   0.8478,   0.1500,
    //               10.0267,   0.5535,   0.5983,   0.1500,
    //               10.1965,   0.2389,   0.6749,   0.1500,
    //              -10.0208,   0.3964,   0.5857,   0.1000,
    //               10.0546,   0.2951,   0.6132,   0.1000,
    //              -10.1062,   0.2444,   0.5897,   0.1300,
    //              -10.0998,   0.3062,   0.5387,   0.1300,
    //               10.1908,   0.5290,   1.0016,   0.2000,
    //               10.2106,   0.4602,   0.6915,   0.2500};
    rclcpp::Time now;
    rclcpp::Time start_time;
    std::chrono::steady_clock::time_point wall_start_time;
    int task_counter = 0;

    void timer_callback()
    {
            auto message = std_msgs::msg::Float64MultiArray();
            
            /*double read_goal[2][7] = {0.81677043, 0.30963624, 0.02739802, -2.00244147, -0.01147922, 2.31379045, 1.63616331,
            -0.05557562, 0.44384545, -0.84215059, -1.9876659, 0.4252202, 2.253303, -0.34089625};*/
            /*double read_goal[2][7] = {0.414479, 0.222319, 0.0105534, -1.90789, -0.0267946, 1.41467, 1.4211,
            -0.451887, 0.284814, -0.192141, -1.83234, 0.12998, 1.38528, 0.498353};*/
            //θf = [0.0 −2.3 −1.1 −1.2 −1.2 0.5]′ 
            //θf = [3.0 −1.6 −1.7 −1.7 −1.7 1.0]′ 
            double read_goal[2][6] = {0.0, -2.3, -1.1, -1.2, -1.2, 0.5, 
                                      2.0, -1.6, -1.7, -1.7, -1.7, 1.0};
            //double read_goal[2][6] = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 
            //1.57, -1.57, 0.0, 0.0, 0.0, 0.0};
                            
            double solver_weights[10] = {20.0, 20.0, 15.0, 15.0, 10.0, 10.0, 1.400, 1.0, 2200.0, 10000.0};
            

            //******************* get_min_dist **********************
            // double min_dist[7] = {10000, 10000, 10000, 10000, 10000, 10000, 10000};
            // double end_effector_pose[3] = {0.0};
            // double spheres_dist[7] = {0.0};
            // double smallest_dist = get_min_dist_arr(current_position, human_sphere,
            //                          min_dist, spheres_dist, end_effector_pose);

            //******************* *********** **********************

            //******************* get_min_velocity **********************
            // double max_vell_current[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // double max_linear_vell_current = get_spheres_velocity(current_position, current_velocity, max_vell_current);
            //******************* *********** **********************

            // check distance to goal
            double j_error = abs(goal_position[0]-current_position[0]);
            std::cout << "pose_error1:";
            for (int i=1;i<6;i++) {
                if (abs(goal_position[i]-current_position[i]) > j_error) 
                    j_error = abs(goal_position[i]-current_position[i]);
                std::cout << abs(goal_position[i]-current_position[i]) << " ";
            }
                std::cout << std::endl;
                std::cout << goal_position[0] << " " << goal_position[1] << " " << goal_position[2] << " "<< goal_position[3] << " " << goal_position[4] << " " << goal_position[5] << std::endl;
                std::cout << current_position[0] << " " << current_position[1] << " " << current_position[2] << " "<< current_position[3] << " " << current_position[4] << " " << current_position[5] << std::endl;
                if (j_error<0.05) {
                    currently_selected_goal = (currently_selected_goal + 1) % 2;
                    if (currently_selected_goal == 0) {
                        now = this->get_clock()->now();
                        MyPerformanceFile << now.seconds() << "," << now.nanoseconds() << "," << task_counter << std::endl;
                        task_counter = task_counter + 1;
                    }
                }
            //
            for (int i=0;i<6;i++) goal_position[i] = read_goal[currently_selected_goal][i];
            
            //
            double tracking_goal[60] = {0};
            /*for (int i=0;i<10;i++) {
                for (int j=0;j<7;j++) tracking_goal[i*7+j]=this->goal_queue[i*7+j];
            }*/
            double cgoal[3] = {0};
            double results[22];
            std::vector<double> full_trajectory(19 * number_of_current_steps, 0.0);

            nmpc_solver->solve_my_mpc(current_position,
                current_velocity,
                // human_sphere, 
                goal_position, tracking_goal, cgoal, results, solver_weights, full_trajectory.data());
            
            //std::cout << "Selected_goal" << currently_selected_goal << std::endl;
            //******************* get_min_velocity **********************
            // double max_vell[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double selected_vels[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            //std::cout << "Result:" << results[0] << std::endl;
            for (int i=0;i<6;i++) selected_vels[i] = results[i + 6];
            // double max_linear_vell = get_spheres_velocity(current_position, selected_vels, max_vell);
            //std::cout << "S1:" << max_vell[0] << " " << max_linear_vell << std::endl;
            //*********************************************************
            // double lin_vell_scale = rescale_velocities(min_dist, spheres_dist, max_vell, selected_vels);
            //std::cout << "S2:" << lin_vell_scale << std::endl;
            
            message.data.clear();
            for (int i=0; i<6;++i)
            {
                message.data.push_back(selected_vels[i]);
            }

            RCLCPP_INFO(this->get_logger(), "calc time [%.2f ms], j_error [%.4f], ocp_cost [%.6f]",
                        results[13], j_error, results[14]);
            now = this->get_clock()->now();
            
            // solver_stats_to_high.csv column order:
            //  1 sec, 2 nsec, 3 sqp_iter, 4 exec_time_ms, 5 ocp_cost, 6 nan_marker, 7 goal_id,
            //  8..13 q1..q6 (current_position),
            // 14..19 qd1..qd6 (current_velocity),
            // 20..25 q_goal1..q_goal6,
            // 26..31 vel_cmd1..vel_cmd6 (selected_vels = predicted x[6:12] at stage 0),
            // 32..37 u1..u6 (solver control input utraj[0:6], joint accelerations).
            MyFile << now.seconds() << "," << now.nanoseconds() << "," << results[12] << "," << results[13] << ","
            << results[14] << "," << results[15] << ","
            // << smallest_dist << "," 
            << currently_selected_goal << "," 
            << current_position[0] << "," << current_position[1] << "," << current_position[2] << "," 
            << current_position[3] << "," << current_position[4] << "," << current_position[5] << "," 
            << current_velocity[0] << "," << current_velocity[1] << "," << current_velocity[2] << "," 
            << current_velocity[3] << "," << current_velocity[4] << "," << current_velocity[5] << "," 
            << goal_position[0] << "," << goal_position[1] << "," << goal_position[2] << "," 
            << goal_position[3] << "," << goal_position[4] << "," << goal_position[5] << ","
            << selected_vels[0] << "," << selected_vels[1] << "," << selected_vels[2] << "," 
            << selected_vels[3] << "," << selected_vels[4] << "," << selected_vels[5] << ","
            << results[16] << "," << results[17] << "," << results[18] << ","
            << results[19] << "," << results[20] << "," << results[21] << ","
            // << human_sphere[0] << "," << human_sphere[1] << "," << human_sphere[2] << "," 
            // << human_sphere[3] << "," << human_sphere[4] << "," << human_sphere[5] << "," 
            // << human_sphere[6] << "," << human_sphere[7] << "," << human_sphere[8] << "," 
            // << human_sphere[9] << "," << human_sphere[10] << "," << human_sphere[11] << "," 
            // << human_sphere[12] << "," << human_sphere[13] << "," << human_sphere[14] << "," 
            // << human_sphere[15] << "," << human_sphere[16] << "," << human_sphere[17] << "," 
            // << human_sphere[18] << "," << human_sphere[19] << "," << human_sphere[20] << "," 
            // << human_sphere[21] << "," << human_sphere[22] << "," << human_sphere[23] << "," 
            // << human_sphere[24] << "," << human_sphere[25] << "," << human_sphere[26] << "," 
            // << human_sphere[27] << "," << human_sphere[28] << "," << human_sphere[29] << "," 
            // << human_sphere[30] << "," << human_sphere[31] << "," << human_sphere[32] << "," 
            // << human_sphere[33] << "," << human_sphere[34] << "," << human_sphere[35] << "," 
            // << human_sphere[36] << "," << human_sphere[37] << "," << human_sphere[38] << "," 
            // << human_sphere[39] << "," << human_sphere[40] << "," << human_sphere[41] << "," 
            // << human_sphere[42] << "," << human_sphere[43] << "," << human_sphere[44] << "," 
            // << human_sphere[45] << "," << human_sphere[46] << "," << human_sphere[47] << "," 
            // << human_sphere[48] << "," << human_sphere[49] << "," << human_sphere[50] << "," 
            // << human_sphere[51] << "," << human_sphere[52] << "," << human_sphere[53] << "," 
            // << human_sphere[54] << "," << human_sphere[55] << ","
            << std::endl;
            
            if (bypass_low_solver) {
                direct_velocity_publisher_->publish(message);
            } else {
                std_msgs::msg::Float64MultiArray joint_vel_values;
                joint_vel_values.data.clear();
                joint_vel_values.data.push_back(number_of_current_steps); 
                for (int i = 0; i < 7 * number_of_current_steps; i++) joint_vel_values.data.push_back(full_trajectory[i]);
                trajectory_publisher_->publish(joint_vel_values);
            }
            const double elapsed_wall_time =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_time).count();
            if (elapsed_wall_time > experiment_duration)
            {
                RCLCPP_INFO(this->get_logger(), "End of experiment (wall time %.2f s / %.2f s)",
                            elapsed_wall_time, experiment_duration);
                rclcpp::shutdown();
                return;
            }

            
            /*message.data.clear();
            int n_steps = 10;
            message.data.push_back(n_steps); // n_steps
            for (int j=0; j<n_steps;++j)
            {
                for (int i=0; i<6;++i) message.data.push_back(read_goal[currently_selected_goal][i]);
                message.data.push_back(0.025);
            }
            trajectory_publisher_->publish(message);*/
            
            //trajectory_publisher_->publish(message);
        //}
        count_++;
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::vector<std::string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
      for (int i=0;i<6;i++) {
          for (int j=0;j<6;j++) {
              if (msg->name[i]==joint_names[j]) {
                this->current_position[j] = msg->position[i];
                this->current_velocity[j] = msg->velocity[i];
              }
          }
      }
    }
    
    // void topic_human_callback(const std_msgs::msg::Float64MultiArray msg)
    // {
    //   for (int i=0;i<56;i++) this->human_sphere[i] = msg.data[i];
    // }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr direct_velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_human_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_goal_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    std::shared_ptr<my_NMPC_solver> nmpc_solver;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotCommandPublisher>();
    node->wait_for_start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    MyFile.close();
    return 0;
}
