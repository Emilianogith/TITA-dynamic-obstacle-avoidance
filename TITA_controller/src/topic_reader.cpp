#include <fstream>
#include <iomanip> 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/joint_state.hpp>




class TopicReader : public rclcpp::Node
{
public:
    TopicReader() : Node("topic_reader")
    {
        // Subscriber for /odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/perception/visual_slam/tracking/Odometry", 10,
            std::bind(&TopicReader::odom_callback, this, std::placeholders::_1));

        // Subscriber for /wheeled_odometry
        wheeled_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/perception/wheel/tracking/Odometry", 10,
            std::bind(&TopicReader::wheeled_odometry_callback, this, std::placeholders::_1));


        // Subscriber for /imu
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/perception/sensor/camera/imu", 10,
            std::bind(&TopicReader::imu_callback, this, std::placeholders::_1));


        // Subscriber for /joint_states
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/tita3233836/joint_states", 10,
            std::bind(&TopicReader::joint_states_callback, this, std::placeholders::_1));

            
        // open log files
        slam_odom_log_.open("slam_odom.txt");
        wheel_odom_log_.open("wheel_odom.txt");
        imu_log_.open("imu_log.txt");
        joint_state_log_.open("joint_state_log.txt");
    }

private:
   
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        // Position
        auto pos = msg->pose.pose.position;
        RCLCPP_INFO(this->get_logger(),
            "[/odom] Position -> x: %.2f, y: %.2f, z: %.2f",
            pos.x, pos.y, pos.z);

        // // Orientation (quaternion)
        // auto ori = msg->pose.pose.orientation;
        // RCLCPP_INFO(this->get_logger(),
        //     "[/odom] Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
        //     ori.x, ori.y, ori.z, ori.w);


        slam_odom_log_ 
            << "Timestamp: " << std::fixed << std::setprecision(9) << (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) << "\n"
            << "Frame ID: " << msg->header.frame_id << "\n"
            << "Child Frame ID: " << msg->child_frame_id << "\n"
            << "Position: x=" << msg->pose.pose.position.x
            << ", y=" << msg->pose.pose.position.y
            << ", z=" << msg->pose.pose.position.z << "\n"
            << "Orientation: x=" << msg->pose.pose.orientation.x
            << ", y=" << msg->pose.pose.orientation.y
            << ", z=" << msg->pose.pose.orientation.z
            << ", w=" << msg->pose.pose.orientation.w << "\n"
            << "Linear Velocity: x=" << msg->twist.twist.linear.x
            << ", y=" << msg->twist.twist.linear.y
            << ", z=" << msg->twist.twist.linear.z << "\n"
            << "Angular Velocity: x=" << msg->twist.twist.angular.x
            << ", y=" << msg->twist.twist.angular.y
            << ", z=" << msg->twist.twist.angular.z << "\n"
            << "----------------------------------------" << std::endl;
    }


    void wheeled_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Position
        // auto pos = msg->pose.pose.position;
        // RCLCPP_INFO(this->get_logger(),
        //     "[/odom] Position -> x: %.2f, y: %.2f, z: %.2f",
        //     pos.x, pos.y, pos.z);


        wheel_odom_log_ << "Timestamp: " << std::fixed << std::setprecision(9) << (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) << "\n"
            << "Frame ID: " << msg->header.frame_id << "\n"
            << "Child Frame ID: " << msg->child_frame_id << "\n"
            << "Position: x=" << msg->pose.pose.position.x
            << ", y=" << msg->pose.pose.position.y
            << ", z=" << msg->pose.pose.position.z << "\n"
            << "Orientation: x=" << msg->pose.pose.orientation.x
            << ", y=" << msg->pose.pose.orientation.y
            << ", z=" << msg->pose.pose.orientation.z
            << ", w=" << msg->pose.pose.orientation.w << "\n"
            << "Linear Velocity: x=" << msg->twist.twist.linear.x
            << ", y=" << msg->twist.twist.linear.y
            << ", z=" << msg->twist.twist.linear.z << "\n"
            << "Angular Velocity: x=" << msg->twist.twist.angular.x
            << ", y=" << msg->twist.twist.angular.y
            << ", z=" << msg->twist.twist.angular.z << "\n"
            << "----------------------------------------" << std::endl;

    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(),
        //     "Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
        //     msg->orientation.x,
        //     msg->orientation.y,
        //     msg->orientation.z,
        //     msg->orientation.w
        // );

        // RCLCPP_INFO(this->get_logger(),
        //     "Angular Velocity -> x: %.3f, y: %.3f, z: %.3f",
        //     msg->angular_velocity.x,
        //     msg->angular_velocity.y,
        //     msg->angular_velocity.z
        // );

        // RCLCPP_INFO(this->get_logger(),
        //     "Linear Acceleration -> x: %.3f, y: %.3f, z: %.3f",
        //     msg->linear_acceleration.x,
        //     msg->linear_acceleration.y,
        //     msg->linear_acceleration.z
        // );

        imu_log_ << "Timestamp: " << std::fixed << std::setprecision(9) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " "
            << "orientation: " << msg->orientation.x << " " << msg->orientation.y << " "
            << msg->orientation.z << " " << msg->orientation.w << " "
            << "angular_velocity: " << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << " "
            << "linear_acceleration: " << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z
            << std::endl;
    }


    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
    RCLCPP_INFO(this->get_logger(), "[/joint_states] Received Joint States");

    for (size_t i = 0; i < msg->name.size(); ++i)
        {
            std::string joint_name = msg->name[i];
            double position = i < msg->position.size() ? msg->position[i] : 0.0;
            double velocity = i < msg->velocity.size() ? msg->velocity[i] : 0.0;
            double effort = i < msg->effort.size() ? msg->effort[i] : 0.0;

            // RCLCPP_INFO(this->get_logger(),
            //     "Joint %s -> Pos: %.3f, Vel: %.3f, Eff: %.3f",
            //     joint_name.c_str(), position, velocity, effort);
                
            joint_state_log_ <<std::fixed << std::setprecision(9) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " "
            << joint_name << ":  pos: "
            << position << " vel: "
            << velocity << " effort: "
            << effort << std::endl;
        }

        joint_state_log_ << "----------------------------------------" << std::endl;

    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheeled_odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    std::ofstream slam_odom_log_;
    std::ofstream wheel_odom_log_;
    std::ofstream imu_log_;
    std::ofstream joint_state_log_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicReader>());
    rclcpp::shutdown();
    return 0;
}
