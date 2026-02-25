#include <chrono>
#include <functional>
#include <memory>

#include "ur_ros2_driver/ur_realtime_communication.h"
#include "ur_ros2_driver/ur_communication.h"
#include "ur_ros2_driver/do_output.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;


class Ros2Wrapper : public rclcpp::Node
{
  public:
    Ros2Wrapper(UrRealtimeCommunication* rt_interface_pointer, UrCommunication* sec_interface_pointer)
    : Node("ur_driver")
    {
		speed_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions speed_options;
		speed_options.callback_group = speed_cb_group_;

		urscript_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions urscript_options;
		urscript_options.callback_group = urscript_cb_group_;

		timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		pubtimer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		rt_interface=rt_interface_pointer;
		sec_interface=sec_interface_pointer;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		sec_interface->start();
		firmware_version_ = sec_interface->robot_state_->getVersion();
		rt_interface->robot_state_->setVersion(firmware_version_);
		// if (!rt_interface->start()){
		// 	break;
		// }
		rt_interface->start();
		ip_addr_ = rt_interface->getLocalIp();
		print_debug(
				"Listening on " + ip_addr_ + ":" + std::to_string(reverse_port)
						+ "\n");
		
		speed_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
		"/my_UR5/velocity_command", 1, std::bind(&Ros2Wrapper::speedInterface, this, _1),speed_options);
		urscript_sub_ = this->create_subscription<std_msgs::msg::String>(
		"ur_driver/URScript", 1, std::bind(&Ros2Wrapper::urscript_callback, this, _1),urscript_options);
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
		timer_ = this->create_wall_timer(8ms, std::bind(&Ros2Wrapper::timer_callback, this),timer_cb_group_);
		publish_timer_ = this->create_wall_timer(10ms, std::bind(&Ros2Wrapper::pub_callback, this),pubtimer_cb_group_);

    }

	~Ros2Wrapper() {
		}

  private:

	void speedInterface(const std_msgs::msg::Float64MultiArray & msg) const {
		for (int i=0;i<6;i++) current_velocity[i] = msg.data[i];
		// rt_interface->setSpeed(current_velocity[0], current_velocity[1], current_velocity[2], current_velocity[3], current_velocity[4], current_velocity[5], 5.0);

		// print_debug(
		// 		"Velocity received " + std::to_string(current_velocity[0]) + std::to_string(current_velocity[4])
		// 				+ "\n");
	}

	void urscript_callback(const std_msgs::msg::String & msg) const {
		rt_interface->addCommandToQueue(msg.data);
	}
	void pub_callback()
	{	
		std::vector<double> qd_actual_;
		qd_actual_=rt_interface->robot_state_->getQdActual();
		q_actual_=rt_interface->robot_state_->getQActual();
		sensor_msgs::msg::JointState msg;    // Message to be published
		// msg.header.frame_id = "";           // Empty frame ID

		msg.name={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};		
		msg.header.stamp = this->get_clock()->now();    // Assign time

		msg.position={q_actual_[0],q_actual_[1],q_actual_[2],q_actual_[3],q_actual_[4],q_actual_[5]};
		msg.velocity={qd_actual_[0],qd_actual_[1],qd_actual_[2],qd_actual_[3],qd_actual_[4],qd_actual_[5]};

		publisher_->publish(msg);
	}
	void timer_callback()
	{	
		rt_interface->setSpeed(current_velocity[0], current_velocity[1], current_velocity[2], current_velocity[3], current_velocity[4], current_velocity[5], 1.0);
	}
	
	mutable std::string ip_addr_;
	mutable double firmware_version_;
	mutable unsigned int reverse_port = 50001;
	mutable double servoj_time;
	mutable double current_velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	mutable std::vector<double> qd_actual_;
	mutable std::vector<double> q_actual_;

	rclcpp::CallbackGroup::SharedPtr speed_cb_group_;
	rclcpp::CallbackGroup::SharedPtr urscript_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
	rclcpp::CallbackGroup::SharedPtr pubtimer_cb_group_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr speed_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urscript_sub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr publish_timer_;
	mutable UrRealtimeCommunication* rt_interface;
	mutable UrCommunication* sec_interface;

};



int main(int argc, char * argv[])
{
	bool use_sim_time = false;
	std::string host = "192.168.1.2";
	std::condition_variable rt_msg_cond_;
	std::condition_variable msg_cond_;
	UrRealtimeCommunication* rt_interface_;
	UrCommunication* sec_interface_;
	unsigned int safety_count_max = 12;
	rt_interface_ = new UrRealtimeCommunication(rt_msg_cond_, host,
		safety_count_max);
	sec_interface_ = new UrCommunication(msg_cond_, host);
	
	rclcpp::init(argc, argv);

	auto my_node = std::make_shared<Ros2Wrapper>(rt_interface_, sec_interface_);
	rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(my_node);
	executor.spin();
	// rclcpp::spin(std::make_shared<Ros2Wrapper>(rt_interface_, sec_interface_)); //host, reverse_port

	delete rt_interface_;
    delete sec_interface_;

	rclcpp::shutdown();
	return 0;
}
