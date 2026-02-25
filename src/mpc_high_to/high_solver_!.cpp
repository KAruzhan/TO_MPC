#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <thread>

using namespace std::chrono;

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/SetModelConfiguration.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <rosgraph_msgs/Clock.h>

#include "acados_solver_holder.cpp"

using namespace std;

ofstream myfile;
ofstream myperffile;

double p_control = 1.0000;
double gazebo_time;
int gripper = 0;

bool is_my_simulation = true;



float dist_v(Eigen::Vector3f v, Eigen::Vector3f w){
    return (v-w).norm();
}

Eigen::MatrixXf get_cpose(float q1, float q2, float q3, float q4, float q5, float q6){
Eigen::MatrixXf mat(3,7);
mat <<
-0.0679*sin(q1),0.2125*cos(q1)*cos(q2) - 0.13585*sin(q1),0.425*cos(q1)*cos(q2) - 0.08405*sin(q1),0.425*cos(q1)*cos(q2) - 0.01615*sin(q1) + 0.1294*cos(q1)*cos(q2)*cos(q3) - 0.1294*cos(q1)*sin(q2)*sin(q3),0.425*cos(q1)*cos(q2) - 0.01615*sin(q1) + 0.2589*cos(q1)*cos(q2)*cos(q3) - 0.2589*cos(q1)*sin(q2)*sin(q3),0.425*cos(q1)*cos(q2) - 0.06265*sin(q1) + 0.39225*cos(q1)*cos(q2)*cos(q3) - 0.39225*cos(q1)*sin(q2)*sin(q3),0.425*cos(q1)*cos(q2) - 0.10915*sin(q1) - 0.0411*cos(q5)*sin(q1) + 0.0411*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - 0.09465*cos(q2 + q3)*cos(q1)*sin(q4) - 0.09465*sin(q2 + q3)*cos(q1)*cos(q4) + 0.39225*cos(q1)*cos(q2)*cos(q3) - 0.39225*cos(q1)*sin(q2)*sin(q3),
0.0679*cos(q1),0.13585*cos(q1) + 0.2125*cos(q2)*sin(q1),0.08405*cos(q1) + 0.425*cos(q2)*sin(q1),0.01615*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.1294*sin(q1)*sin(q2)*sin(q3) + 0.1294*cos(q2)*cos(q3)*sin(q1),0.01615*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.2589*sin(q1)*sin(q2)*sin(q3) + 0.2589*cos(q2)*cos(q3)*sin(q1),0.06265*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.39225*sin(q1)*sin(q2)*sin(q3) + 0.39225*cos(q2)*cos(q3)*sin(q1),0.10915*cos(q1) + 0.0411*cos(q1)*cos(q5) + 0.425*cos(q2)*sin(q1) - 0.39225*sin(q1)*sin(q2)*sin(q3) + 0.0411*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 0.09465*cos(q2 + q3)*sin(q1)*sin(q4) - 0.09465*sin(q2 + q3)*cos(q4)*sin(q1) + 0.39225*cos(q2)*cos(q3)*sin(q1),
0.089159000000000002139621813057602,0.089159000000000002139621813057602 - 0.2125*sin(q2),0.089159000000000002139621813057602 - 0.425*sin(q2),0.089159000000000002139621813057602 - 0.425*sin(q2) - 0.1294*sin(q2 + q3),0.089159000000000002139621813057602 - 0.425*sin(q2) - 0.2589*sin(q2 + q3),0.089159000000000002139621813057602 - 0.425*sin(q2) - 0.39225*sin(q2 + q3),0.09465*sin(q2 + q3)*sin(q4) - 0.425*sin(q2) - sin(q5)*(0.0411*cos(q2 + q3)*sin(q4) + 0.0411*sin(q2 + q3)*cos(q4)) - 0.09465*cos(q2 + q3)*cos(q4) - 0.39225*sin(q2 + q3) + 0.089159000000000002139621813057602;
    return mat;
}





// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 

    // Data Members 
    ros::Publisher chatter_pub;
    ros::Publisher goal_state;

    double robot_spheres[7] =  {0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.10};
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
    double solver_weights[10] = {20.0, 20.0, 15.0, 15.0, 10.0, 10.0, 1.400, 10.0, 2200.0, 10000.0};

    double probabilities_predicted[2]={0};
    double human_sphere_predicted[1120]={0}; 

    double goal[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
    double comand_vel[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
    double joint_position[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
    double joint_speed[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
  
    // Member Functions() 

    void change_goal(double new_goal[],int n) 
    { 
       for (int i=0; i<n; i++) goal[i] = new_goal[i];
       ROS_INFO("Goal set to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
    goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]); 
    }

    void change_weights(const std_msgs::Float64MultiArray weight_message) 
    { 
       for (int i=0; i<10; i++) solver_weights[i] = weight_message.data[i];
       ROS_INFO("New weigths: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
    solver_weights[0], solver_weights[1], solver_weights[2], solver_weights[3], solver_weights[4], solver_weights[5], solver_weights[6], solver_weights[7], solver_weights[8], solver_weights[9]); 
    }

    void change_obstacles_msg(const std_msgs::Float64MultiArray obstacle_data) 
    { 
       for (int i=0; i<56; i++) human_sphere[i] = obstacle_data.data[i];
    }

    void change_goal_msg(const std_msgs::Float64MultiArray joint_pose_values) 
    { 
       for (int i=0; i<6; i++) goal[i] = joint_pose_values.data[i];
       ROS_INFO("New goal set to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
       goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]); 
    }

    void change_states_msg(const sensor_msgs::JointState::ConstPtr& msg) 
    { 
       if (gripper == 1) {
           for (int i=0; i<6; i++) joint_position[i] = msg->position[i+1];
           for (int i=0; i<6; i++) joint_speed[i] = msg->velocity[i+1];
       }
       else {
           for (int i=0; i<6; i++) joint_position[i] = msg->position[i];
           for (int i=0; i<6; i++) joint_speed[i] = msg->velocity[i];
       }
       //for (int i=0; i<7; i++) joint_position[i] = msg->position[i];
    }

    void SendVelocity(const std_msgs::Float64MultiArray joint_vel_values){
        chatter_pub.publish(joint_vel_values);
    return;
    }

}; 

/*
void gazebo_time_msg(const rosgraph_msgs::Clock::ConstPtr& msg){
    gazebo_time = msg->clock.toSec();
    return;
}
*/


int main(int argc, char **argv)
{
  
  myfile.open("data_high.csv", ios::out);
  myperffile.open("data_perf.csv", ios::out); 
  ros::init(argc, argv, "joint_controller_high");

  ros::NodeHandle n;
  if (ros::param::has("/is_my_simulation")) {
      ros::param::get("/is_my_simulation", is_my_simulation);
      if (is_my_simulation) {
          gripper = 1;
          p_control = 12.0000;
      }
      else {
          gripper = 0;
          p_control = 1.0000;
              ROS_INFO("It is real robot\n\n");
      }
     ROS_INFO("Param set!!!!\n\n");
  }


  ROS_INFO("Node Started");
  //--------------------------------
  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/my_gen3/mpc_high_positions", 1);
  my_follower.goal_state = n.advertise<std_msgs::String>("/my_gen3/goal_status", 1);

  ros::Publisher Gripper_command = n.advertise<std_msgs::Float64>("/my_gen3/moveit_gripper/command", 1);


  ROS_INFO("Goal default to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
        my_follower.goal[0], my_follower.goal[1], my_follower.goal[2],
        my_follower.goal[3], my_follower.goal[4], my_follower.goal[5]);

  double read_goal[2][6] = {0.0, -2.3, -1.1, -1.2, -1.2, 0.5,
                            3.0, -1.6, -1.7, -1.7, -1.3, 1.0};

  int row_index_len = 2;

  double static_goal[6] = {0.0, -2.3, -1.1, -1.2, -1.2, 0.5};

  //--------------------------------
  
  // turn off  
  ros::Subscriber joint_goal = n.subscribe("/my_gen3/joint_goal", 1, &GoalFollower::change_goal_msg, &my_follower);
  //------
  ros::Subscriber human_status = n.subscribe("/gazebo/mpc_spheres", 1, &GoalFollower::change_obstacles_msg, &my_follower);

  ros::Subscriber joint_status = n.subscribe("/joint_states", 1, &GoalFollower::change_states_msg, &my_follower);
  //ros::Subscriber gazebo_time_listener = n.subscribe("/clock", 1, gazebo_time_msg);


  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  gazebo_msgs::SetModelConfiguration srv_msg;

  ros::Subscriber weight_pub = n.subscribe("/simulation_weight", 1, &GoalFollower::change_weights, &my_follower);
  

  int number_of_current_steps = 5;
  my_NMPC_solver myMpcSolver=my_NMPC_solver(1,number_of_current_steps);
  
  my_NMPC_solver second_myMpcSolver=my_NMPC_solver(1,number_of_current_steps+1);

  std_msgs::Float64MultiArray joint_vel_values;
  double cgoal[3];

  // Here is an entty point of the simulation  
  //boost::shared_ptr<std_msgs::Float64MultiArray const> temp;
  //temp =  ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/gazebo/mpc_spheres", ros::Duration(5));
  ros::Publisher Sim_results = n.advertise<std_msgs::Float64>("/sim_results", 1);
  std_msgs::Float64 sim_results_value = std_msgs::Float64();
  
  
  boost::shared_ptr<std_msgs::String const> simulation_status;
  bool started = false;

  jump:
  ROS_INFO("Jumped");
  started = false;
  std::string start_label = "Start";
  std_msgs::String start_dummy;
  while (!started) {
    simulation_status =  ros::topic::waitForMessage<std_msgs::String>("/simulation_status", ros::Duration(10));
    if(simulation_status != NULL){
      start_dummy = *simulation_status;
      if (start_dummy.data == start_label) started = true;
    }
  }
  ROS_INFO("Start recieved %f ", ros::Time::now().toSec());
  
  double my_weights[10] = {20.0, 20.0, 15.0, 15.0, 10.0, 10.0, 1.400, 10.0, 2200.0, 10000.0};
  for (int i=0; i<10; i++) my_weights[i] = my_follower.solver_weights[i]; 
  ros::Time time = ros::Time::now();
  double start_time_main = time.toSec();
  double wall_time = time.toSec();
  double internal_summ = 0.0;
  int internal_counter = 0;

  // Big loop
  double loop_duration = 840.65417; // traint cropped; 1306.07531; // <- futt train 329.88889;//840.65417;//353.511612;//// no pauses
  double start_motion_time = 10.00;
  double stop_human_time = loop_duration;
  for (int big_loop_iteration=0;big_loop_iteration<1;big_loop_iteration++) {
    start_motion_time = big_loop_iteration*0.5 + 15.0;
    stop_human_time = big_loop_iteration*loop_duration + loop_duration;
    ROS_INFO("Time: %.3f, %.3f", start_motion_time, stop_human_time);


    //** Low level Loop
    int row_index = 0;
    double loop_start_time = 0;
    ros::Rate goto_loop(20);
    ros::Duration(0.50).sleep();
    // loop for initial position
    while (wall_time-start_time_main < start_motion_time + stop_human_time - loop_duration){
      // prepare to send commands
      joint_vel_values.data.clear();
      for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(0.0);
      for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(0.0);
      for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(static_goal[i]);
      
      Eigen::MatrixXf cgoal_mat = get_cpose(static_goal[0], static_goal[1], 
           static_goal[2], static_goal[3], static_goal[4], 
           static_goal[5]);
      cgoal[0] = cgoal_mat.coeff(0, 7);
      cgoal[1] = cgoal_mat.coeff(1, 7);
      cgoal[2] = cgoal_mat.coeff(2, 7);
      for (int i = 0; i < 3; i++) joint_vel_values.data.push_back(cgoal[i]);
      my_follower.SendVelocity(joint_vel_values);
      ros::spinOnce();
      time = ros::Time::now();
      wall_time = time.toSec();
      goto_loop.sleep();
    }; // End 

    int task = -1;
    int task_started = 0;
    ros::Rate loop_rate(4);
    while (wall_time-start_time_main < stop_human_time)
    {
      time = ros::Time::now();
      wall_time = time.toSec();
      // change to arrive check. add ~1.5s before next entrance
      if (row_index==0) {
        if (task_started == 0) {
          task = task + 1;
          loop_start_time = wall_time-start_time_main;
        }
        task_started = 1;
      }

      //*******************

      // Goal reference position
      double current_joint_position[6] = {0.0};
      double current_human_position[56];
      //double current_human_position_predicted[1120]={0};
      double current_joint_goal[6];
      for (int i = 0; i < 6; ++i) current_joint_position[ i ] = my_follower.joint_position[ i ];
      for (int i = 0; i < 6; ++i) current_joint_goal[ i ] = read_goal[row_index][i];
      for (int i = 0; i < 56; ++i) current_human_position[ i ] = my_follower.human_sphere[ i ];

      double probability_first = 1.0;
      double probability_second = 1.0;
      if (my_follower.probabilities_predicted[0]>0.0001){
        probability_first = my_follower.probabilities_predicted[0]/(my_follower.probabilities_predicted[0] + my_follower.probabilities_predicted[1]);
        probability_second = my_follower.probabilities_predicted[1]/(my_follower.probabilities_predicted[0] + my_follower.probabilities_predicted[1]);
      }

      Eigen::MatrixXf cgoal_mat = get_cpose(read_goal[row_index][0], read_goal[row_index][1], 
               read_goal[row_index][2], read_goal[row_index][3], read_goal[row_index][4], 
               read_goal[row_index][5]);
      cgoal[0] = cgoal_mat.coeff(0, 7);
      cgoal[1] = cgoal_mat.coeff(1, 7);
      cgoal[2] = cgoal_mat.coeff(2, 7);
      // end Cartesian Goal
  
      //ROS_INFO("Pose %f %f %f %f %f %f", current_joint_position[0], current_joint_position[1], current_joint_position[2], current_joint_position[3], current_joint_position[4], current_joint_position[5]);
      //ROS_INFO("Goal %f %f %f %f %f %f", current_joint_goal[0], current_joint_goal[1], current_joint_goal[2], current_joint_goal[3], current_joint_goal[4], current_joint_goal[5]);
      double result[16]={0.0};
      double trajectory_first[66]={0.0};
      double full_trajectory[70]={0.0};
      
      
      double second_result[16]={0.0};
      double second_trajectory_first[66]={0.0};
      double second_full_trajectory[70]={0.0};
      
      //int status = 0;
      
      
      int number_of_last_steps = number_of_current_steps;
      
      //thread th1(&my_NMPC_solver::solve_my_mpc, &second_myMpcSolver, current_joint_position, current_human_position, current_joint_goal, cgoal, second_result, second_trajectory_first, second_full_trajectory);
      
      //thread th1(&my_NMPC_solver::solve_my_mpc, &myMpcSolver, current_joint_position, current_human_position, current_joint_goal, cgoal, result, trajectory_first, full_trajectory);
      
      int status=myMpcSolver.solve_my_mpc(current_joint_position, current_human_position, current_joint_goal, cgoal, result, trajectory_first, full_trajectory);

      //th1.join();

      double lattest_mpc_time = result[2];
      //double lower_limit_mpc_time = [];
      //double upper_limit_mpc_time = [];
      
      switch(number_of_current_steps) {
      case 3:
        if (lattest_mpc_time>20.0*2 || status==4) number_of_current_steps=4;
        break;
      case 4:
        if (lattest_mpc_time>30.0*2 || status==4) number_of_current_steps=5;
        else if (lattest_mpc_time<20.0*2) number_of_current_steps=3;
        break;
      case 5:
        if (lattest_mpc_time>41.0*2 || status==4) number_of_current_steps=6;
        else if (lattest_mpc_time<30.0*2) number_of_current_steps=4;
        break;
      case 6:
        if (lattest_mpc_time>55.0*2 || status==4) number_of_current_steps=7;
        else if (lattest_mpc_time<41.0*2) number_of_current_steps=5;
        break;
      case 7:
        if (lattest_mpc_time>64.0*2 || status==4) number_of_current_steps=8;
        else if (lattest_mpc_time<55.0*2) number_of_current_steps=6;
        break;
      case 8:
        if (lattest_mpc_time>110.0*2 || status==4) number_of_current_steps=9;
        else if (lattest_mpc_time<64.0*2) number_of_current_steps=7;
        break;
      case 9:
        if (lattest_mpc_time>145.0*2 || status==4) number_of_current_steps=10;
        else if (lattest_mpc_time<110.0*2) number_of_current_steps=8;
        break;
      case 10:
        if (lattest_mpc_time<145.0*2) number_of_current_steps=9;
        break;
      //default:
        // code block
    }
    
    myMpcSolver.reset_solver();
    myMpcSolver=my_NMPC_solver(2,number_of_current_steps);
    if (status==4) for (int i=0; i<12; i++) result[i] = 0.0;




      //*********************** Apply control ********************************

      // Check if arrived
      std_msgs::String goal_state_msg;
      std::stringstream ss;
      float max_diff = 0;
      for (int i = 0; i < 6; ++i) {
          if (abs(current_joint_position[i] - current_joint_goal[i]) > max_diff) {
              max_diff = abs(current_joint_position[i] - current_joint_goal[i]); 
          }
      }
      // real robot (max_diff < 0.01)
      //ROS_INFO("max_diff %f",max_diff);
      if (max_diff < 0.05) {
          ss << "Arrived";
          if (row_index==(row_index_len-1) && task_started==1) {
              task_started = 0;
              double perf_record = wall_time-start_time_main - loop_start_time;
              if (task!=0) {
                internal_counter = internal_counter+1;
                internal_summ = internal_summ + perf_record;
                myperffile << "Performance " << big_loop_iteration <<" "<< perf_record << " " << task << endl;
              }
          }
          int64_t timestamp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
          myperffile << "Step "  << timestamp << " "<< wall_time-start_time_main << " " << big_loop_iteration << " " << task << " " << row_index << endl;
          row_index = (row_index+1)%row_index_len;
          ros::Duration(0.20).sleep();
      }
      else ss << "Following";
      goal_state_msg.data = ss.str();
      my_follower.goal_state.publish(goal_state_msg);

      //******************* get_min_dist **********************
      float local_val = 10000;
      double smallest_dist = 10000;
      double min_dist[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000};
      Eigen::MatrixXf mat2 = get_cpose(my_follower.joint_position[0], my_follower.joint_position[1], 
             my_follower.joint_position[2], my_follower.joint_position[3], my_follower.joint_position[4], 
             my_follower.joint_position[5]);


      for (int j = 0; j<7; j++) {
        Eigen::Vector3f w;
        w = mat2.col(j+1).transpose();
        for (int i = 0; i < 14; i++) {
            Eigen::Vector3f p(my_follower.human_sphere[i*4+0],my_follower.human_sphere[i*4+1],my_follower.human_sphere[i*4+2]);
            local_val = dist_v(w, p) - my_follower.robot_spheres[j] - my_follower.human_sphere[i*4+3];
            if (min_dist[j] > local_val) min_dist[j] = local_val;
        }
        if (smallest_dist > min_dist[j]) smallest_dist = min_dist[j];
      }
      
      // prepare to send commands
      joint_vel_values.data.clear();
      /*for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(current_joint_position[i]);
      for (int i = 6; i < 20; i++) joint_vel_values.data.push_back(result[i]);
      for (int i = 0; i < 3; i++) joint_vel_values.data.push_back(cgoal[i]);
      my_follower.SendVelocity(joint_vel_values);
      */
      joint_vel_values.data.push_back(number_of_last_steps);
      for (int i = 0; i < 70; i++) joint_vel_values.data.push_back(full_trajectory[i]);
      my_follower.SendVelocity(joint_vel_values);

      // save data to file
      std::stringstream save2file;
      if (myfile.is_open())
      { // mpc_time, dt,  KKT, is_infisible, smallest_dist
        myfile << wall_time << " " << result[2] << " " << result[4] << " " << number_of_last_steps << endl;
      }
      else cout << "Unable to open file";

      
      loop_rate.sleep();
      ros::spinOnce();
    }
    //** end of Low level Loop
  }
  ROS_INFO("Mean Performance %d, %f, %f", internal_counter, internal_summ, internal_summ/internal_counter);
  sim_results_value.data = internal_summ/internal_counter;
  Sim_results.publish(sim_results_value);
  // Here write GOTO
  goto jump;
  myperffile.close();
  myfile.close();
  return 0;
}

