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

using namespace std::chrono;

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/SetModelConfiguration.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <rosgraph_msgs/Clock.h>

#include "acados_solver_holder.cpp"

using namespace std;

ofstream myfile;
ofstream myperffile;

ofstream trajectory_file;

double p_control = 1.0000;
double gazebo_time;
double pybullet_time;
int gripper = 0;

bool is_my_simulation = true;


  static const long long int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
  static const long long int casadi_s1[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  /* calc_y:(i0[6])->(o0[12]) */
  int calc_y(const double arg[1][6], double res[1][12]) {
    double a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a4, a5, a6, a7, a8, a9;
    a0=3.9224999999999999e-01;
    a1=arg[0]? arg[0][0] : 0;
    a2=cos(a1);
    a3=arg[0]? arg[0][1] : 0;
    a4=cos(a3);
    a5=(a2*a4);
    a6=arg[0]? arg[0][2] : 0;
    a7=cos(a6);
    a8=(a5*a7);
    a9=sin(a3);
    a2=(a2*a9);
    a10=sin(a6);
    a11=(a2*a10);
    a8=(a8-a11);
    a11=(a0*a8);
    a12=4.2499999999999999e-01;
    a13=(a12*a5);
    a14=-1.1970000000000000e-01;
    a15=sin(a1);
    a16=(a14*a15);
    a13=(a13-a16);
    a16=1.3585000000000000e-01;
    a17=(a16*a15);
    a13=(a13-a17);
    a11=(a11+a13);
    a13=9.2999999999999999e-02;
    a17=(a13*a15);
    a11=(a11-a17);
    a17=9.4649999999999998e-02;
    a18=cos(a6);
    a2=(a2*a18);
    a6=sin(a6);
    a5=(a5*a6);
    a2=(a2+a5);
    a5=arg[0]? arg[0][3] : 0;
    a19=cos(a5);
    a20=(a2*a19);
    a21=sin(a5);
    a22=(a8*a21);
    a20=(a20+a22);
    a22=(a17*a20);
    a11=(a11-a22);
    a22=4.1099999999999998e-02;
    a23=sin(a5);
    a2=(a2*a23);
    a5=cos(a5);
    a8=(a8*a5);
    a2=(a2-a8);
    a8=arg[0]? arg[0][4] : 0;
    a24=sin(a8);
    a25=(a2*a24);
    a26=cos(a8);
    a27=(a15*a26);
    a25=(a25+a27);
    a27=(a22*a25);
    a11=(a11-a27);
    if (res[0]!=0) res[0][0]=a11;
    a11=cos(a1);
    a26=(a11*a26);
    a1=sin(a1);
    a9=(a1*a9);
    a27=(a9*a18);
    a1=(a1*a4);
    a4=(a1*a6);
    a27=(a27+a4);
    a4=(a27*a23);
    a28=(a1*a7);
    a9=(a9*a10);
    a28=(a28-a9);
    a9=(a28*a5);
    a4=(a4-a9);
    a9=(a4*a24);
    a26=(a26-a9);
    a9=(a22*a26);
    a13=(a13*a11);
    a29=(a0*a28);
    a14=(a14*a11);
    a1=(a12*a1);
    a14=(a14+a1);
    a16=(a16*a11);
    a14=(a14+a16);
    a29=(a29+a14);
    a13=(a13+a29);
    a27=(a27*a19);
    a28=(a28*a21);
    a27=(a27+a28);
    a28=(a17*a27);
    a13=(a13-a28);
    a9=(a9+a13);
    if (res[0]!=0) res[0][1]=a9;
    a9=sin(a3);
    a6=(a9*a6);
    a3=cos(a3);
    a18=(a3*a18);
    a6=(a6-a18);
    a19=(a6*a19);
    a3=(a3*a10);
    a7=(a9*a7);
    a3=(a3+a7);
    a21=(a3*a21);
    a19=(a19+a21);
    a17=(a17*a19);
    a21=8.9159000000000002e-02;
    a12=(a12*a9);
    a21=(a21-a12);
    a0=(a0*a3);
    a21=(a21-a0);
    a17=(a17+a21);
    a3=(a3*a5);
    a6=(a6*a23);
    a3=(a3-a6);
    a24=(a3*a24);
    a22=(a22*a24);
    a17=(a17-a22);
    if (res[0]!=0) res[0][2]=a17;
    a17=cos(a8);
    a2=(a2*a17);
    a8=sin(a8);
    a15=(a15*a8);
    a2=(a2-a15);
    a15=arg[0]? arg[0][5] : 0;
    a22=cos(a15);
    a6=(a2*a22);
    a23=sin(a15);
    a5=(a20*a23);
    a6=(a6+a5);
    if (res[0]!=0) res[0][3]=a6;
    a25=(-a25);
    if (res[0]!=0) res[0][4]=a25;
    a25=sin(a15);
    a2=(a2*a25);
    a15=cos(a15);
    a20=(a20*a15);
    a2=(a2-a20);
    if (res[0]!=0) res[0][5]=a2;
    a4=(a4*a17);
    a11=(a11*a8);
    a4=(a4+a11);
    a11=(a4*a22);
    a8=(a27*a23);
    a11=(a11+a8);
    if (res[0]!=0) res[0][6]=a11;
    if (res[0]!=0) res[0][7]=a26;
    a4=(a4*a25);
    a27=(a27*a15);
    a4=(a4-a27);
    if (res[0]!=0) res[0][8]=a4;
    a3=(a3*a17);
    a22=(a3*a22);
    a23=(a19*a23);
    a22=(a22-a23);
    if (res[0]!=0) res[0][9]=a22;
    a24=(-a24);
    if (res[0]!=0) res[0][10]=a24;
    a3=(a3*a25);
    a19=(a19*a15);
    a3=(a3+a19);
    if (res[0]!=0) res[0][11]=a3;
    return 0;
  }

double my_trapez2(double q0, double qf, double t_array[10], double v_current, double v_limit, double a_limit, double p[10], double pd[10], double pdd[10]) {
    double dist = fabs(qf - q0);
    int direction = 0;
    if (dist==(qf-q0)) direction = 1;
    else direction = -1;
    double time_rise = (v_limit*direction - v_current) / a_limit;
    double S_rise = time_rise * (v_limit*direction + v_current) / 2.0;
    double time_fall = v_limit / a_limit;
    double S_fall = time_fall * v_limit / 2.0;
    double S_triag = S_rise + S_fall;
    double time_stop = v_current / a_limit;
    double S_stop = time_stop * v_current / 2;
    double time_linear = 0;

    if (S_triag > dist) {
        // triangular
        double v_triag_max = sqrt((S_triag * (2 * a_limit) + v_current * v_current) / 2.0);
        double t_peak = v_triag_max / a_limit;
        time_rise = t_peak;
        time_linear = 0;
        double time_fall = t_peak;
    } else {
        // trapezoidal
        double t_linear = (dist - S_triag) / v_limit;
        time_linear = t_linear;
    }

    //printf("time: %lf %lf %lf\n", time_rise, time_linear, time_fall);
    //printf("Total time: %lf\n", time_rise + time_linear + time_fall);

    // Sample from trajectory
    //double p[10]; // Adjust the array size as needed
    //double pd[10];
    //double pdd[10];

    double T = time_rise + time_linear + time_fall;

    for (int i = 0; i < 10; i++) {
        double tk = t_array[i];
        double pk, pdk, pddk;
        if (tk < 0) {
            pk = q0;
            pdk = 0;
            pddk = 0;
        } else if (tk <= time_rise) {
            // initial blend
            pk = q0 + (v_current + a_limit*direction * tk) / 2 * tk;
            pdk = v_current + a_limit*direction * tk;
            pddk = a_limit*direction;
        } else if (tk <= (time_rise + time_linear)) {
            // linear motion
            pk = q0 + (v_current + a_limit*direction * time_rise) / 2 * time_rise + v_limit*direction * (tk - time_rise);
            pdk = v_limit*direction;
            pddk = 0;
        } else if (tk <= T) {
            // final blend
            pk = q0 + (v_current + a_limit*direction * time_rise) / 2 * time_rise + v_limit*direction * time_linear + (v_limit*direction + a_limit*direction * (tk - (time_rise + time_linear))) / 2 * (tk - (time_rise + time_linear));
            pdk = v_limit*direction - a_limit*direction * (tk - (time_rise + time_linear));
            pddk = -a_limit*direction;
        } else {
            pk = qf;
            pdk = 0;
            pddk = 0;
        }
        p[i] = pk;
        pd[i] = pdk;
        pdd[i] = pddk;
    }

    // Print the results (you can modify this part as needed)
    /*printf("Position: ");
    for (int i = 0; i < 10; i++) {
        printf("%lf ", p[i]);
    }
    printf("\n");

    printf("Velocity: ");
    for (int i = 0; i < 10; i++) {
        printf("%lf ", pd[i]);
    }
    printf("\n");

    printf("Acceleration: ");
    for (int i = 0; i < 10; i++) {
        printf("%lf ", pdd[i]);
    }
    printf("\n");*/
    return T;
}

double my_trapez(double q0, double qf, double t_array[10], double V, double p[10], double pd[10], double pdd[10]) {
    double T = 0, max_t = 0;
    int length = 10;
    for (int i = 0; i < length; ++i) {
        if (t_array[i] > max_t) {
            max_t = t_array[i];
        }
    }
    T = max_t; // trajectory duration

    // 1) calculate or check velocity limit
    if (V == 0) {
        // if velocity not specified, compute it
        V = (qf - q0) / T * 1.5;
    } else {
        V = fabs(V) * ((qf - q0) < 0 ? -1 : 1);
        if (fabs(V) < (fabs(qf - q0) / T)) {
            printf("V too small\n");
            V = (qf - q0) / T * 1.5;
        } else if (fabs(V) > (2 * fabs(qf - q0) / T)) {
            printf("V too big\n");
            V = (qf - q0) / T * 1.5;
        }
    }

    // 2) Calculate blend time
    double tb, a;
    if (V == 0) {
        tb = 100000000.0;//INFINITY;
        a = 0;
    } else {
        tb = (q0 - qf + V * T) / V;
        a = V / tb;
    }

    // 3) Sample from trajectory
    for (int i = 0; i < length; ++i) {
        double tk = t_array[i];
        double pk, pdk, pddk;
        if (tk < 0) {
            pk = q0;
            pdk = 0.0;
            pddk = 0.0;
        } else if (tk <= tb) {
            // initial blend
            pk = q0 + a / 2 * pow(tk, 2);
            pdk = a * tk;
            pddk = a;
        } else if (tk <= (T - tb)) {
            // linear motion
            pk = (qf + q0 - V * T) / 2 + V * tk;
            pdk = V;
            pddk = 0.0;
        } else if (tk <= T) {
            // final blend
            pk = qf - a / 2 * pow(T, 2) + a * T * tk - a / 2 * pow(tk, 2);
            pdk = a * T - a * tk;
            pddk = -a;
        } else {
            pk = qf;
            pdk = 0.0;
            pddk = 0.0;
        }
        p[i] = pk;
        pd[i] = pdk;
        pdd[i] = pddk;
    }

    return a;
}



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

void pybullet_time_msg(const std_msgs::Float64 msg){
    pybullet_time = msg.data;
    return;
}


int main(int argc, char **argv)
{
  
  myfile.open("data_high.csv", ios::out);
  myperffile.open("data_perf.csv", ios::out);
  trajectory_file.open("trajectory_init.csv", ios::out); 
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

  ros::Publisher pause_state = n.advertise<std_msgs::String>("/simulation_pause", 1);

  ros::Publisher Gripper_command = n.advertise<std_msgs::Float64>("/my_gen3/moveit_gripper/command", 1);


  ROS_INFO("Goal default to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
        my_follower.goal[0], my_follower.goal[1], my_follower.goal[2],
        my_follower.goal[3], my_follower.goal[4], my_follower.goal[5]);

  double read_goal[2][6] = {0.0, -2.3, -1.1, -1.2, -1.2, 0.5,
                            3.0, -1.6, -1.7, -1.7, -1.3, 1.0};
                            
  double read_goal_target[2][12] = {-0.752151, 0.124043, 0.354528, 0.512065, 0.104531, -0.852562, -0.817941, 0.362358, -0.446843, 0.262224, 0.926159, 0.271051,
  0.479769, -0.189748, 0.463229, -0.684775, 0.23284, 0.690556, 0.623488, -0.303393, 0.720566, 0.377287, 0.923979, 0.0625833};

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
  
  ros::Publisher high_level_trajectory = n.advertise<nav_msgs::Path>("mpc_solver/high_level_trajectory", 1);
  

  const int number_of_current_steps = 20;
  const int number_of_sqp_iters = 5;
  my_NMPC_solver myMpcSolver=my_NMPC_solver(number_of_sqp_iters,number_of_current_steps);

  std_msgs::Float64MultiArray joint_vel_values;
  double cgoal[3];
  float c_cgoal[3];
  float c_pose[3];
  double test[3];

  // Here is an entty point of the simulation  
  //boost::shared_ptr<std_msgs::Float64MultiArray const> temp;
  //temp =  ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/gazebo/mpc_spheres", ros::Duration(5));
  ros::Publisher Sim_results = n.advertise<std_msgs::Float64>("/sim_results", 1);
  std_msgs::Float64 sim_results_value = std_msgs::Float64();
  
  
  boost::shared_ptr<std_msgs::String const> simulation_status;
  boost::shared_ptr<std_msgs::Float64 const> sim_time_msg;
  bool started = false;

  jump:
  ROS_INFO("Jumped");
  started = false;
  std::string start_label = "Start";

  std_msgs::String pause_label;
  pause_label.data = "Pause";
  std_msgs::String unpause_label;
  unpause_label.data = "Unpause";

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
  double loop_duration = 1800; //840.65417; // traint cropped; 1306.07531; // <- futt train 329.88889;//840.65417;//353.511612;//// no pauses
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
    
    int is_paused = 0;

    int task = -1;
    int task_started = 0;
    ros::Rate loop_rate(4);
    ROS_INFO("Core started!\n");
    while (wall_time-start_time_main < stop_human_time)
    {
      time = ros::Time::now();
      wall_time = time.toSec();
      sim_time_msg =  ros::topic::waitForMessage<std_msgs::Float64>("/pyBullyt_time", ros::Duration(999));
      std_msgs::Float64 time_dummy;
      if(sim_time_msg != NULL){
        time_dummy = *sim_time_msg;
        wall_time = double(time_dummy.data);
      }
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
      double current_joint_velocity[6] = {0.0};
      double current_human_position[56];
      //double current_human_position_predicted[1120]={0};
      double current_joint_goal[6];
      double target_goal[18]={0.0};
      
      for (int i = 0; i < 6; ++i) current_joint_position[ i ] = my_follower.joint_position[ i ];
      for (int i = 0; i < 6; ++i) current_joint_velocity[ i ] = my_follower.joint_speed[ i ];
      for (int i = 0; i < 6; ++i) current_joint_goal[ i ] = read_goal[row_index][i];
      for (int i = 0; i < 12; ++i) target_goal[ i ] = read_goal_target[row_index][i];
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
      cgoal[0] = cgoal_mat(0, 7);
      cgoal[1] = cgoal_mat(1, 7);
      cgoal[2] = cgoal_mat(2, 7);
      c_cgoal[0] = cgoal_mat(0, 6);
      c_cgoal[1] = cgoal_mat(1, 6);
      c_cgoal[2] = cgoal_mat(2, 6);
      // end Cartesian Goal
  
      ROS_INFO("Pose %f %f %f %f %f %f", current_joint_position[0], current_joint_position[1], current_joint_position[2], current_joint_position[3], current_joint_position[4], current_joint_position[5]);
      ROS_INFO("C goal: %f %f %f",c_cgoal[0],c_cgoal[1],c_cgoal[2]);
      //cout << cgoal_mat << endl;
      
      //******************* get_min_dist **********************
      float local_val = 10000;
      double smallest_dist = 10000;
      double min_dist[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000};
      Eigen::MatrixXf mat2 = get_cpose(my_follower.joint_position[0], my_follower.joint_position[1], 
             my_follower.joint_position[2], my_follower.joint_position[3], my_follower.joint_position[4], 
             my_follower.joint_position[5]);
             
      c_pose[0] = mat2.coeff(0, 6);
      c_pose[1] = mat2.coeff(1, 6);
      c_pose[2] = mat2.coeff(2, 6);
      
      ROS_INFO("C pose: %f %f %f",c_pose[0],c_pose[1],c_pose[2]);

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
      
      
      float cartesian_error = 10000;
      cartesian_error = sqrt((c_pose[0] - c_cgoal[0])*(c_pose[0] - c_cgoal[0]) + 
                             (c_pose[1] - c_cgoal[1])*(c_pose[1] - c_cgoal[1]) +
                             (c_pose[2] - c_cgoal[2])*(c_pose[2] - c_cgoal[2]));
      
      
      //ROS_INFO("Goal %f %f %f %f %f %f", current_joint_goal[0], current_joint_goal[1], current_joint_goal[2], current_joint_goal[3], current_joint_goal[4], current_joint_goal[5]);
      double result[16]={0.0};
      double full_trajectory[6*(20+1)]={0.0};
      double full_velocities[7*20]={0.0};
      
      double flat_trajectory[140]={0.0};
      
      double a_calc1 = 0.0;double a_calc2= 0.0;double a_calc3 = 0.0;double a_calc4 = 0.0;double a_calc5 = 0.0;double a_calc6 = 0.0;
      //double t_array[] = {0.0, 0.33333333, 0.66666667, 1.0, 1.33333333, 1.66666667, 2.0, 2.33333333, 2.66666667, 3.0};
      double t_array[] = {0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5};
      double V = 1.35;
      double p_1[10] = {0.0};double p_2[10] = {0.0};double p_3[10] = {0.0};double p_4[10] = {0.0};double p_5[10] = {0.0};double p_6[10] = {0.0};
      double pd_1[10] = {0.0};double pd_2[10] = {0.0};double pd_3[10] = {0.0};double pd_4[10] = {0.0};double pd_5[10] = {0.0};double pd_6[10] = {0.0};
      double pdd_1[10] = {0.0};double pdd_2[10] = {0.0};double pdd_3[10] = {0.0};double pdd_4[10] = {0.0};double pdd_5[10] = {0.0};double pdd_6[10] = {0.0};
      // my_trapez(double q0, double qf, double t_array[10], double V, double p[10], double pd[10], double pdd[10])
      /*a_calc1 = my_trapez(current_joint_position[0], current_joint_goal[0],t_array, 0.0, p_1, pd_1, pdd_1);
      a_calc2 = my_trapez(current_joint_position[1], current_joint_goal[1],t_array, 0.0, p_2, pd_2, pdd_2);
      a_calc3 = my_trapez(current_joint_position[2], current_joint_goal[2],t_array, 0.0, p_3, pd_3, pdd_3);
      a_calc4 = my_trapez(current_joint_position[3], current_joint_goal[3],t_array, 0.0, p_4, pd_4, pdd_4);
      a_calc5 = my_trapez(current_joint_position[4], current_joint_goal[4],t_array, 0.0, p_5, pd_5, pdd_5);
      a_calc6 = my_trapez(current_joint_position[5], current_joint_goal[5],t_array, 0.0, p_6, pd_6, pdd_6);*/
      
      //a_calc1 = my_trapez(0.0, -3.0, t_array, v_current, v_limit, a_limit, p_1, pd_1, pdd_1);
      double v_limit = 1.35;
      double a_limit = 4.4;
      a_calc1 = my_trapez2(current_joint_position[0], current_joint_goal[0], t_array, current_joint_velocity[0], v_limit, a_limit, p_1, pd_1, pdd_1);
      a_calc2 = my_trapez2(current_joint_position[1], current_joint_goal[1], t_array, current_joint_velocity[1], v_limit, a_limit, p_2, pd_2, pdd_2);
      a_calc3 = my_trapez2(current_joint_position[2], current_joint_goal[2], t_array, current_joint_velocity[2], v_limit, a_limit, p_3, pd_3, pdd_3);
      a_calc4 = my_trapez2(current_joint_position[3], current_joint_goal[3], t_array, current_joint_velocity[3], v_limit, a_limit, p_4, pd_4, pdd_4);
      a_calc5 = my_trapez2(current_joint_position[4], current_joint_goal[4], t_array, current_joint_velocity[4], v_limit, a_limit, p_5, pd_5, pdd_5);
      a_calc6 = my_trapez2(current_joint_position[5], current_joint_goal[5], t_array, current_joint_velocity[5], v_limit, a_limit, p_6, pd_6, pdd_6);
      
      //ROS_INFO("a_trapez = %f,%f,%f,%f,%f,%f",a_calc1,a_calc2,a_calc3,a_calc4,a_calc5,a_calc6);
      
      double p_array[60] = {0.0};
      for (int i = 0; i < 10; i++) p_array[i] = p_1[i];
      for (int i = 0; i < 10; i++) p_array[i+10] = p_2[i];
      for (int i = 0; i < 10; i++) p_array[i+20] = p_3[i];
      for (int i = 0; i < 10; i++) p_array[i+30] = p_4[i];
      for (int i = 0; i < 10; i++) p_array[i+40] = p_5[i];
      for (int i = 0; i < 10; i++) p_array[i+50] = p_6[i];
      
      
      /*for (int i = 0; i < 10; i++)
      {   
          printf("params %i %f %f %f %f %f %f \n %f %f %f %f %f %f \n", i, p_1[i], p_2[i], p_3[i], p_4[i], p_5[i], p_6[i],
                                                                           pd_1[i], pd_2[i], pd_3[i], pd_4[i], pd_5[i], pd_6[i]);
      }*/
      pd_1[0]=0.0; pd_2[0]=0.0; pd_3[0]=0.0; pd_4[0]=0.0;  pd_5[0]=0.0;  pd_6[0]=0.0; 
      /*for (int i = 1; i < 10; i++)
      {
      	   pd_1[i]=(p_1[i]-p_1[i-1])/0.30; pd_2[i]=(p_2[i]-p_2[i-1])/0.30; pd_3[i]=(p_3[i]-p_3[i-1])/0.30;
      	   pd_4[i]=(p_4[i]-p_4[i-1])/0.30; pd_5[i]=(p_5[i]-p_5[i-1])/0.30; pd_6[i]=(p_6[i]-p_6[i-1])/0.30; 
      }*/
     
      // Publish pause
      if (is_paused) pause_state.publish(pause_label);
      
      
      double arg_input[1][6] = {0.0};
      double arg_res[1][12] = {0.0};
      //double *input = &arg_input;
      //double *res = &arg_res;     
      //int get_fk = calc_y(arg_input, arg_res);
      //ROS_INFO("calc_y: %f %f %f", arg_res[0][0], arg_res[0][1], arg_res[0][2]);
      
      //ROS_INFO("r_1: %f %f %f", arg_res[0][3], arg_res[0][4], arg_res[0][5]);
      //ROS_INFO("r_2: %f %f %f", arg_res[0][6], arg_res[0][7], arg_res[0][8]);
      //ROS_INFO("r_3: %f %f %f", arg_res[0][9], arg_res[0][10], arg_res[0][11]);
      
      //double target_goal[18] = {0.0};
      
      //for (int i=0;i<12;i++) target_goal[i]=arg_res[0][i];
      
      ROS_INFO("calc_y: %f %f %f", target_goal[0], target_goal[1], target_goal[2]);
      
      //if (cartesian_error>0.1) {
          // use MPC for large distances
          int status = 0;
          status=myMpcSolver.solve_my_mpc(current_joint_position, current_joint_velocity, current_human_position, current_joint_goal,
            result, full_trajectory, full_velocities, target_goal);
          // record the trajectory
          trajectory_file << "Pose: "  ;
          for (const auto& value : current_joint_position) trajectory_file << value << " ";
          for (const auto& value : current_joint_velocity) trajectory_file << value << " ";
          for (const auto& value : current_joint_goal) trajectory_file << value << " ";
          trajectory_file << endl;
          trajectory_file << "Solve: ";
          for (const auto& value : full_trajectory) trajectory_file << value << " ";
          for (const auto& value : full_velocities) trajectory_file << value << " ";
          trajectory_file << endl;
          
          
          //ROS_INFO("KKT %f; Status %i",result[14], status);
          //if (result[15] > 0.0 ) printf("\n\n NaN Detected ##################################################\n\n");

          //ROS_INFO("Start plotting 1");
          if (status > 0 ) {
              //ROS_INFO("Destroying solver object");
              myMpcSolver.reset_solver();
              myMpcSolver=my_NMPC_solver(number_of_sqp_iters,number_of_current_steps);
              //for (int i=0; i<12; i++) result[i] = 0.0;
              //ROS_INFO("Solver recreated");
          }
          if (full_velocities[6]<0.0501) {
            for (int i=0;i<10;i++) {
              for (int j=0;j<6;j++) full_trajectory[i*6+j] = current_joint_goal[j];
              full_velocities[i*7+6] = 0.01;
            }
            for (int j=0;j<6;j++) full_trajectory[10*6+j]=current_joint_goal[j]; 
          }
          
          
      /*}
      else { // send goal around goalpoint
          for (int i=0;i<10;i++) {
              for (int j=0;j<6;j++) full_trajectory[i*6+j] = current_joint_goal[j];
              full_velocities[i*7+6] = 0.01;
          }
      }*/
      // Publish unpause and start timer
      if (is_paused) pause_state.publish(unpause_label);
      double new_loop_time = 0.0;
      double after_pause_time = 0.0;
      time = ros::Time::now();
      after_pause_time = time.toSec();

      /*}*/
      /*
      ROS_INFO("Start plotting");
      double cgoal_trejectory[33];
      if (status == 0) 
      {
          for (int i=0;i<10;i++) 
          {
              Eigen::MatrixXf cgoal_trajectory_mat = get_cpose(full_trajectory[7*i+0], full_trajectory[7*i+1], 
		           full_trajectory[7*i+2], full_trajectory[7*i+3], full_trajectory[7*i+4], 
		           full_trajectory[7*i+5]);
              cgoal_trejectory[3*i+0] = cgoal_trajectory_mat.coeff(0, 7);
              cgoal_trejectory[3*i+1] = cgoal_trajectory_mat.coeff(1, 7);
              cgoal_trejectory[3*i+2] = cgoal_trajectory_mat.coeff(2, 7);
              ROS_INFO("%i: %f %f %f",i, cgoal_trejectory[3*i+0],cgoal_trejectory[3*i+1],cgoal_trejectory[3*i+2]);
          }
          //ROS_INFO("%f %f %f",cgoal_trejectory[3*9+0],cgoal_trejectory[3*9+1],cgoal_trejectory[3*9+2]);
      }
      nav_msgs::Path path;
      path.header.frame_id="/base";
      geometry_msgs::PoseStamped pose;
      
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "waypoint";
      for (int i=0;i<11;i++) {
      pose.pose.position.x = cgoal_trejectory[3*i+0];
      pose.pose.position.y = cgoal_trejectory[3*i+1];
      pose.pose.position.z = cgoal_trejectory[3*i+2];
      
      path.poses.push_back(pose);
      }
       
      high_level_trajectory.publish(path);
      */
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
      ROS_INFO("max_diff %f, c_dif %f",max_diff, cartesian_error);
      if (cartesian_error < 0.01) {
      //if (max_diff < 0.05 && cartesian_error < 0.01) {
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
      
      // prepare to send commands
      joint_vel_values.data.clear();
      /*for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(current_joint_position[i]);
      for (int i = 6; i < 20; i++) joint_vel_values.data.push_back(result[i]);
      for (int i = 0; i < 3; i++) joint_vel_values.data.push_back(cgoal[i]);
      my_follower.SendVelocity(joint_vel_values);
      */
      joint_vel_values.data.push_back(number_of_current_steps+1);
      //for (int i=0;i<number_of_current_steps;i++) joint_vel_values.data.push_back(full_velocities[7*i+6]); // send only the dt array
      //for (const auto& value : full_trajectory) joint_vel_values.data.push_back(value);
      for (int i=0;i<number_of_current_steps;i++) {
        for (int j=0;j<6;j++) joint_vel_values.data.push_back(full_trajectory[(i)*6+j]);
        joint_vel_values.data.push_back(full_velocities[7*i+6]); // send only the dt array
      }
      for (int j=0;j<6;j++) joint_vel_values.data.push_back(full_trajectory[number_of_current_steps*6+j]);
      joint_vel_values.data.push_back(0.0); // send only the dt array
      my_follower.SendVelocity(joint_vel_values);

      //cout << joint_vel_values << endl;

      // save data to file
      std::stringstream save2file;
      if (myfile.is_open())
      { // mpc_time, KKT, is_infisible, smallest_dist
        myfile << wall_time << " " << result[0] << " " << result[1] << " " << result[2] << " " << result[3] << " " << smallest_dist << " " << row_index << " " <<
            my_follower.joint_position[0] << " " << my_follower.joint_position[1] << " " <<
            my_follower.joint_position[2] << " " << my_follower.joint_position[3] << " " <<
            my_follower.joint_position[4] << " " << my_follower.joint_position[5] << " " <<
            my_follower.joint_speed[0] << " " << my_follower.joint_speed[1] << " " <<
            my_follower.joint_speed[2] << " " << my_follower.joint_speed[3] << " " <<
            my_follower.joint_speed[4] << " " << my_follower.joint_speed[5] << " " << 
my_follower.human_sphere[0] << " " << my_follower.human_sphere[1] << " " << my_follower.human_sphere[2] << " " << my_follower.human_sphere[3] << " " << my_follower.human_sphere[4] << " " << my_follower.human_sphere[5] << " " << my_follower.human_sphere[6] << " " << my_follower.human_sphere[7] << " " << my_follower.human_sphere[8] << " " << my_follower.human_sphere[9] << " " << my_follower.human_sphere[10] << " " << my_follower.human_sphere[11] << " " << my_follower.human_sphere[12] << " " << my_follower.human_sphere[13] << " " << my_follower.human_sphere[14] << " " << my_follower.human_sphere[15] << " " << my_follower.human_sphere[16] << " " << my_follower.human_sphere[17] << " " << my_follower.human_sphere[18] << " " << my_follower.human_sphere[19] << " " << my_follower.human_sphere[20] << " " << my_follower.human_sphere[21] << " " << my_follower.human_sphere[22] << " " << my_follower.human_sphere[23] << " " << my_follower.human_sphere[24] << " " << my_follower.human_sphere[25] << " " << my_follower.human_sphere[26] << " " << my_follower.human_sphere[27] << " " << my_follower.human_sphere[28] << " " << my_follower.human_sphere[29] << " " << my_follower.human_sphere[30] << " " << my_follower.human_sphere[31] << " " << my_follower.human_sphere[32] << " " << my_follower.human_sphere[33] << " " << my_follower.human_sphere[34] << " " << my_follower.human_sphere[35] << " " << my_follower.human_sphere[36] << " " << my_follower.human_sphere[37] << " " << my_follower.human_sphere[38] << " " << my_follower.human_sphere[39] << " " << my_follower.human_sphere[40] << " " << my_follower.human_sphere[41] << " " << my_follower.human_sphere[42] << " " << my_follower.human_sphere[43] << " " << my_follower.human_sphere[44] << " " << my_follower.human_sphere[45] << " " << my_follower.human_sphere[46] << " " << my_follower.human_sphere[47] << " " << my_follower.human_sphere[48] << " " << my_follower.human_sphere[49] << " " << my_follower.human_sphere[50] << " " << my_follower.human_sphere[51] << " " << my_follower.human_sphere[52] << " " << my_follower.human_sphere[53] << " " << my_follower.human_sphere[54] << " " << my_follower.human_sphere[55] << " " << endl;
      }
      /*{ // mpc_time, dt,  KKT, is_infisible, smallest_dist
        // time, mpc_time, dt, solver_status, smallest_dist, dist_to_goal
        myfile << wall_time << " " << result[2] << " " << full_trajectory[6] << " " << status << " " << smallest_dist << " " << max_diff 
        << " " << my_follower.joint_speed[0] << " " << my_follower.joint_speed[1] << " " << my_follower.joint_speed[2] << " " << my_follower.joint_speed[3]
        << " " << my_follower.joint_speed[4] << " " << my_follower.joint_speed[5] << endl;
        //myfile << wall_time << " " << result[2] << " " << result[4] << " " << number_of_last_steps << endl;
      }*/
      else cout << "Unable to open file";

      // Whait that after solution time > 250ms

      /*do {
          time = ros::Time::now();
          new_loop_time = time.toSec();
      } while (new_loop_time< after_pause_time + 0.250);*/
      
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
  trajectory_file.close();
  return 0;
}

