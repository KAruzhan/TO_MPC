#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "acados_solver_holder.hpp"

//#include "kinematics_functions.cpp"

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::ofstream MyFile;

float dist_v(Eigen::Vector3f v, Eigen::Vector3f w){
    return (v-w).norm();
}

double get_min_dist_arr(double joint_position[7], double human_sphere[56], double min_dist[8], double spheres_dist[8], double end_effector_pose[3])
{
    double local_val = 10000;
    double smallest_dist = 10000;
    int number_of_spheres_robot = 8;
    int number_of_spheres_human = 14;
    double robot_spheres[8] = {0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.10, 0.10};

    Eigen::MatrixXf mat2 = get_cpose(joint_position[0], joint_position[1], 
        joint_position[2], joint_position[3], joint_position[4], 
        joint_position[5], joint_position[6]);

    for (int j = 0; j < number_of_spheres_robot-1; j++) {
        Eigen::Vector3f w;
        w = mat2.col(j+1).transpose();
        for (int k = 0; k < number_of_spheres_human; k++) 
        {
            Eigen::Vector3f p(human_sphere[k*4+0],human_sphere[k*4+1],human_sphere[k*4+2]);
            local_val = dist_v(w, p) - robot_spheres[j] - human_sphere[k*4+3];
            if (local_val < min_dist[j]) {
                min_dist[j] = local_val;
                spheres_dist[j] = robot_spheres[j] + human_sphere[k*4+3];
            }
            //std::cout << k << ":" << p << std::endl;
        }
        if (smallest_dist > min_dist[j]) smallest_dist = min_dist[j];
    }
    Eigen::Vector3f end_effector_pose_vect = mat2.col(number_of_spheres_robot-1);

    end_effector_pose[0] = end_effector_pose_vect.x();
    end_effector_pose[1] = end_effector_pose_vect.y();
    end_effector_pose[2] = end_effector_pose_vect.z();
    return smallest_dist;
}


double get_spheres_velocity(double joint_position[7], double joint_velocities[7], double max_vell[8])
{
    Eigen::MatrixXf vell_mat = get_velocity(joint_position[0], joint_position[1], 
        joint_position[2], joint_position[3], joint_position[4],
        joint_position[5], joint_position[6],
        joint_velocities[0], joint_velocities[1], joint_velocities[2],
        joint_velocities[3], joint_velocities[4], joint_velocities[5], joint_velocities[6]);
    double max_linear_vell = 0;
    double temp_linear_vell = 0;
    for (int k=0; k<8; k++) {
        temp_linear_vell = sqrt(
            vell_mat.coeff(k*3 + 0,0)*vell_mat.coeff(k*3 + 0,0) + 
            vell_mat.coeff(k*3 + 1,0)*vell_mat.coeff(k*3 + 1,0) + 
            vell_mat.coeff(k*3 + 2,0)*vell_mat.coeff(k*3 + 2,0)
            );
        max_vell[k] = temp_linear_vell;
        if (max_linear_vell < temp_linear_vell) max_linear_vell = temp_linear_vell;
    }
    return max_linear_vell;
}


double rescale_velocities(double min_dist[8], double spheres_dist[8], double max_vell[8], double joint_vels[7])
{
    //*********************** Apply control ********************************
    double lin_vell_limit_arr[8] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    double lin_vell_scale = 10.0;
    double alpha = 0.75; //v_mpc_lim = sqrt(alpha^2.*((min_dist_arr+spheres_dist).^2-spheres_dist.^2));
    double sqrt_temp_value = 0.00;

    for (int i=0;i<8;i++) {
      sqrt_temp_value = (min_dist[i]+spheres_dist[i])*(min_dist[i]+spheres_dist[i])-spheres_dist[i]*spheres_dist[i];
      if (sqrt_temp_value<0) lin_vell_limit_arr[i] = 0.00000000000000;
      else lin_vell_limit_arr[i] = alpha*sqrt(sqrt_temp_value);

      if (lin_vell_limit_arr[i]<0) lin_vell_limit_arr[i] = 0.00000000000000;
      double temp_scale = (lin_vell_limit_arr[i]/max_vell[i]);
      if (lin_vell_scale>temp_scale) lin_vell_scale = temp_scale;
    }
      
    if (lin_vell_scale<1.0) {
        for (int i = 0; i < 7; i++) joint_vels[i] = joint_vels[i]*lin_vell_scale;
    }
    return lin_vell_scale;
}


class RobotCommandPublisher : public rclcpp::Node
{
public:
    RobotCommandPublisher() : Node("robot_command_publisher"), count_(0)
    {
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocity_controller/commands", 1);

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&RobotCommandPublisher::topic_callback, this, _1));
        
        subscription_human_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/human_pose", 1, std::bind(&RobotCommandPublisher::topic_human_callback, this, _1));

        joint_goal_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/mpc_high_positions", 1, std::bind(&RobotCommandPublisher::change_goal_msg, this, _1));

        timer_ = this->create_wall_timer
        (
            50ms,
            std::bind(&RobotCommandPublisher::timer_callback, this)
        );
        
        MyFile.open("solver_stats.txt"); //, std::ofstream::out | std::ofstream::app);

        nmpc_solver = std::make_shared<my_NMPC_solver>(1);

    }

private:

    double current_position[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double current_velocity[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double goal_position[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double goal_queue[140] = {0.0};
    int currently_selected_goal = 0;
    double human_sphere[56]= {10.0517,   0.5220,   1.0895,   0.1500,
                  10.0658,   0.4526,   0.8624,   0.2500,
                  10.0844,   0.7044,   0.9207,   0.1500,
                  10.2083,   0.3075,   1.0208,   0.1500,
                  10.0556,   0.6289,   0.7595,   0.1500,
                  10.2024,   0.2732,   0.8478,   0.1500,
                  10.0267,   0.5535,   0.5983,   0.1500,
                  10.1965,   0.2389,   0.6749,   0.1500,
                 -10.0208,   0.3964,   0.5857,   0.1000,
                  10.0546,   0.2951,   0.6132,   0.1000,
                 -10.1062,   0.2444,   0.5897,   0.1300,
                 -10.0998,   0.3062,   0.5387,   0.1300,
                  10.1908,   0.5290,   1.0016,   0.2000,
                  10.2106,   0.4602,   0.6915,   0.2500};

    void timer_callback()
    {
            auto message = std_msgs::msg::Float64MultiArray();
            
            double read_goal[2][7] = {3.45117578e-04, -7.87005784e-01, -4.01465964e-05, -2.35303298e+00, -1.96423566e-03,  1.56946516e+00,  7.90385160e-01, 
       1.16670721,  0.70940623,  0.35487472, -1.64929862, -0.30835366, 2.28996984,  0.84475702};
                            
            double solver_weights[11] = {20.0, 20.0, 15.0, 15.0, 15.0, 10.0, 10.0, 1.400, 1.0, 2200.0, 10000.0};

            // move goal queue and update current goal
            advance_goal();

            //******************* get_min_dist **********************
            double min_dist[8] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000};
            double end_effector_pose[3] = {0.0};
            double spheres_dist[8] = {0.0};
            double smallest_dist = get_min_dist_arr(current_position, human_sphere,
                                     min_dist, spheres_dist, end_effector_pose);

            //******************* *********** **********************

            //******************* get_min_velocity **********************
            double max_vell_current[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double max_linear_vell_current = get_spheres_velocity(current_position, current_velocity, max_vell_current);
            //******************* *********** **********************

            // check distance to goal
            double j_error = abs(goal_position[0]-current_position[0]);
            for (int i=1;i<7;i++) {
                if (abs(goal_position[i]-current_position[i])>j_error) j_error = abs(goal_position[i]-current_position[i]);
            }
            if (j_error<0.005) currently_selected_goal = (currently_selected_goal + 1) % 2;
            //
            for (int i=0;i<7;i++) goal_position[i] = read_goal[currently_selected_goal][i];
            //
            double tracking_goal[70] = {0};
            for (int i=0;i<10;i++) {
                for (int j=0;j<7;j++) tracking_goal[i*7+j]=this->goal_queue[i*7+j];
            }
            double cgoal[3] = {0};
            double results[16];

            nmpc_solver->solve_my_mpc(current_position, human_sphere, goal_position, tracking_goal, cgoal, results, solver_weights);
            
            
            //******************* get_min_velocity **********************
            double max_vell[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double selected_vels[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::cout << "Result:" << results[0] << std::endl;
            for (int i=0;i<7;i++) selected_vels[i] = results[i];
            double max_linear_vell = get_spheres_velocity(current_position, selected_vels, max_vell);
            std::cout << "S1:" << max_vell[0] << " " << max_linear_vell << std::endl;
            //*********************************************************
            double lin_vell_scale = rescale_velocities(min_dist, spheres_dist, max_vell, selected_vels);
            std::cout << "S2:" << lin_vell_scale << std::endl;
            message.data.clear();
            for (int i=0; i<7;++i)
            {
                message.data.push_back(selected_vels[i]);
            }
            
            RCLCPP_INFO(this->get_logger(), "calc time [%f], j_error [%f]", results[15], j_error);
            rclcpp::Time now = this->get_clock()->now();
            
            MyFile << now.nanoseconds() << "," << results[15] << "," << j_error << ","
            << goal_position[0] << "," << goal_position[1] << "," << goal_position[2] << "," << goal_position[3] << "," 
            << goal_position[4] << "," << goal_position[5] << "," << goal_position[6] << "," 
            << current_position[0] << "," << current_position[1] << "," << current_position[2] << "," << current_position[3] << "," 
            << current_position[4] << "," << current_position[5] << "," << current_position[6] << "," 
            << std::endl;
            
            velocity_publisher_->publish(message);
        //}
        count_++;
    }

    void change_goal_msg(const std_msgs::msg::Float64MultiArray solution_msg) 
    {   
        constexpr int DOF = 6;             // Degrees of freedom per goal
        constexpr int sample_line = DOF+1; // join position and dt
        constexpr int MAX_STEPS = 20;      // Maximum steps in trajectory
        constexpr double MAX_TIME = 0.5;   // Maximum trajectory time
        constexpr double TIME_STEP = 0.05; // Time interval for remapping
        constexpr double OFFSET = 0.25;    // Offset for trajectory timing

        // n_steps goal_1(6) time_1 goal_2(6) ... goal_n(6)
        int n_steps = static_cast<int>(solution_msg.data[0]); // Number of steps
        //ROS_INFO("Provided dt: %f", solution_msg.data[DOF + 1]);
        // Check if there's a valid solution, otherwise use current position
        if (solution_msg.data[DOF + 1] < 0.005) 
        {
            for (int j = 0; j < MAX_STEPS; ++j) 
            {
                for (int i = 0; i < DOF; ++i) 
                {
                    this->goal_queue[j * DOF + i] = current_position[i];
                }
            }
            for (int i=0; i<DOF; i++) this->goal_position[i] = this->current_position[i];
        }
        else 
        {
            // Limit the steps in command trajectory by time
            std::vector<double> time_vector(MAX_STEPS, 0.0);
            int steps = 0;
            double cumulative_time = 0.0;
            for (int i = 0; i < n_steps && cumulative_time <= MAX_TIME; ++i) 
            {
                cumulative_time += solution_msg.data[7 * i + DOF + 1];
                time_vector[i] = cumulative_time;
                steps = i;
            }

            // Remap trajectory to 0.05s intervals for a 0.5s trajectory
            for (int i=0; i<MAX_STEPS; ++i) 
            {
                double current_mark = i * TIME_STEP + OFFSET; // test with 250 offset first, skipped 1 step
                // Find the correct segment in the time vector
                if (time_vector[steps] < current_mark) 
                {
                    for (int k=0; k<DOF; k++) 
                    {
                        this->goal_queue[i*DOF+k] = solution_msg.data[sample_line*steps+k+1];
                    }
                }
                for (int j=1; j<steps; ++j) 
                {
                    if (time_vector[j] >= current_mark)
                    {
                        int t1 = j-1; int t2 = j;
                        double tau = (current_mark-time_vector[t1])/(time_vector[t2]-time_vector[t1]);
                        // Perform linear interpolation
                        for (int k=0; k<DOF; k++) this->goal_queue[i*DOF+k] = 
                            (1-tau)*solution_msg.data[sample_line*t1+k+1] + 
                            tau*solution_msg.data[sample_line*t2+k+1];
                        break;
                    }
                }
            }
        }
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::vector<std::string> joint_names = {"ur5_joint1","ur5_joint2","ur5_joint3","ur5_joint4","ur5_joint5","ur5_joint6","ur5_joint7"};
      for (int i=0;i<7;i++) {
          for (int j=0;j<7;j++) {
              if (msg->name[i]==joint_names[j]) {
                this->current_position[j] = msg->position[i];
                this->current_velocity[j] = msg->velocity[i];
              }
          }
      }
    }
    
    void topic_human_callback(const std_msgs::msg::Float64MultiArray msg)
    {
      for (int i=0;i<56;i++) this->human_sphere[i] = msg.data[i];
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_human_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_goal_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    std::shared_ptr<my_NMPC_solver> nmpc_solver;

    void advance_goal()
    {
        // shift goals from the last 
        int NUMBER_OF_GOALS = 20;
        int GOAL_SIZE = 7;
        for (int j=0; j<NUMBER_OF_GOALS-1; j++) {
            for (int i=0; i<7; i++){ 
                this->goal_queue[j*GOAL_SIZE+i] = this->goal_queue[(j+1)*GOAL_SIZE+i]; 
            }
        }
        for (int i=0; i<GOAL_SIZE; i++) this->goal_position[i] = this->goal_queue[(NUMBER_OF_GOALS-1)*GOAL_SIZE+i];
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotCommandPublisher>());
    rclcpp::shutdown();
    MyFile.close();
    return 0;
}
