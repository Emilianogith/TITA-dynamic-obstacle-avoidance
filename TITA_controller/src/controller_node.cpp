#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>




class RobotController : public rclcpp::Node
{
public:
    RobotController()
    : Node("robot_controller"), count_(0)
    {
        
        joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/tita3233836/joint_states", 10);
        timer_joint_state_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotController::publish_joint_state, this));
    }

private:

    void publish_joint_state()
    {
        auto msg = sensor_msgs::msg::JointState();

        // Fill the header
        msg.header.stamp = this->now();
     

        // Define joint names
        msg.name = {
            "joint_left_leg_1", "joint_left_leg_2", "joint_left_leg_3", "joint_left_leg_4",
            "joint_right_leg_1", "joint_right_leg_2", "joint_right_leg_3", "joint_right_leg_4"
        };

        msg.effort.resize(msg.name.size());

        for (size_t i = 0; i < msg.name.size(); ++i)
        {
            if (i == 3 || i == 7){
                msg.effort[i] = 0.01 *count_++; 
            }else{
                msg.effort[i] = 0.0; 
            }
        }

        joint_state_->publish(msg);
    }


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
    rclcpp::TimerBase::SharedPtr timer_joint_state_;

    int count_;
};




int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
