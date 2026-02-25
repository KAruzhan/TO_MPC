#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class RobotCommandPublisher : public rclcpp::Node
{
public:
    RobotCommandPublisher() : Node("robot_command_publisher"), count_(0)
    {
        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/my_UR5/position_command", 10);
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/my_UR5/velocity_command", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&RobotCommandPublisher::topic_callback, this, _1));

        timer_ = this->create_wall_timer
        (
            50ms,
            std::bind(&RobotCommandPublisher::timer_callback, this)
        );

    }
    


private:

    void timer_callback()
    {
        /*if(count_ % 2 == 0)
        {
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {0.0, -3.57, 4.57, 0.0, 0.0, 0.0};
            RCLCPP_INFO(this->get_logger(), "Publishing position command: [%f, %f, %f, %f, %f, %f]", 
                message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5]);
            position_publisher_->publish(message);
        }
        else
        {*/
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double p_value = 0.5; // Proportional controller coefficient
            for (int i=0;i<6;i++) message.data[i] = p_value*(this->goal_position[i] - this->current_position[i]);
            RCLCPP_INFO(this->get_logger(), "Publishing velocity command: [%f, %f, %f, %f, %f, %f]",
                message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5]);
            velocity_publisher_->publish(message);
        //}
        count_++;
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Pose: [%f, %f, %f, %f, %f, %f]", msg->position[0], msg->position[1], msg->position[2]
                                                    , msg->position[3], msg->position[4], msg->position[5]);
      for (int i=0;i<6;i++) this->current_position[i] = msg->position[i];
    }


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    double current_position[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double goal_position[6] = {0.0, 0.0, 0.0, 0.0, -1.57, 0.0}; 

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}