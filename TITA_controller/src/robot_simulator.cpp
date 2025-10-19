#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <random>

double get_random(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}




class RobotSimulatort : public rclcpp::Node
{
public:
    RobotSimulatort()
    : Node("robot_simulator"), count_(0)
    {
        
        odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/perception/visual_slam/tracking/Odometry", 10);
        timer_odom_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotSimulatort::publish_odom, this));

        wheeled_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/perception/wheel/tracking/Odometry", 10);
        timer_wheeled_odom_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotSimulatort::publish_wheel_odom, this));

        imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/perception/sensor/camera/imu", 10);
        timer_imu_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotSimulatort::publish_imu, this));

        joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/tita3233836/joint_states", 10);
        timer_joint_state_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotSimulatort::publish_joint_state, this));
    }

private:

    void publish_odom()
    {
        
        auto odom_msg = nav_msgs::msg::Odometry();

        // Header
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Position
        odom_msg.pose.pose.position.x = 1.0 + 0.1 * count_++;
        odom_msg.pose.pose.position.y = 2.0;
        odom_msg.pose.pose.position.z = 0.0;

        // Orientation (quaternion)
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        // Linear velocity
        odom_msg.twist.twist.linear.x = 0.1;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        // Angular velocity
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.1;

        odom_->publish(odom_msg);

        RCLCPP_INFO(this->get_logger(), "Published odometry: x=%.2f", odom_msg.pose.pose.position.x);
    }



    void publish_wheel_odom()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "wheel";

        // Fake pose (simulate robot moving forward)
        msg.pose.pose.position.x = get_random(0.0, 5.0);
        msg.pose.pose.position.y = get_random(-0.5, 0.5);
        msg.pose.pose.position.z = 0.0;

        // Orientation as quaternion (no rotation)
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;

        // Simulated linear velocity
        msg.twist.twist.linear.x = get_random(0.0, 1.0);
        msg.twist.twist.linear.y = 0.0;
        msg.twist.twist.linear.z = 0.0;

        // Simulated angular velocity
        msg.twist.twist.angular.z = get_random(-0.2, 0.2);

        wheeled_odom_->publish(msg);
    }



    void publish_imu()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // Orientation (no rotation)
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;

        // Angular velocity (gyro)
        msg.angular_velocity.x = get_random(-0.1, 0.1);
        msg.angular_velocity.y = get_random(-0.1, 0.1);
        msg.angular_velocity.z = get_random(-0.2, 0.2);

        // Linear acceleration (accelerometer)
        msg.linear_acceleration.x = get_random(-0.2, 0.2);
        msg.linear_acceleration.y = get_random(-0.2, 0.2);
        msg.linear_acceleration.z = 9.8 + get_random(-0.1, 0.1);  // Gravity plus noise

        imu_->publish(msg);
    }



    void publish_joint_state()
    {
        auto msg = sensor_msgs::msg::JointState();

        // Fill the header
        msg.header.stamp = this->now();
        msg.header.frame_id = "";

        // Define joint names
        msg.name = {
            "joint_left_leg_1", "joint_left_leg_2", "joint_left_leg_3", "joint_left_leg_4",
            "joint_right_leg_1", "joint_right_leg_2", "joint_right_leg_3", "joint_right_leg_4"
        };

        // Generate random positions and velocities
        msg.position.resize(msg.name.size());
        msg.velocity.resize(msg.name.size());
        msg.effort.resize(msg.name.size()); // Optional: fill with zeros

        for (size_t i = 0; i < msg.name.size(); ++i)
        {
            msg.position[i] = get_random(-3.14, 3.14);   // Simulate joint angles (in radians)
            msg.velocity[i] = get_random(-1.0, 1.0);     // Simulate velocity
            msg.effort[i] = 0.0;                         // No effort simulation here
        }

        joint_state_->publish(msg);
    }


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
    rclcpp::TimerBase::SharedPtr timer_odom_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheeled_odom_;
    rclcpp::TimerBase::SharedPtr timer_wheeled_odom_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::TimerBase::SharedPtr timer_imu_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
    rclcpp::TimerBase::SharedPtr timer_joint_state_;

    int count_;
};




int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSimulatort>());
  rclcpp::shutdown();
  return 0;
}
