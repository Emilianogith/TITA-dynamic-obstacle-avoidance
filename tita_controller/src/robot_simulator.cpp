#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <random>


#include <fstream>
#include <sstream>
#include <vector>
#include <string>

double get_random(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}


struct ImuData {
    double orientation_x, orientation_y, orientation_z, orientation_w;
    double angular_x, angular_y, angular_z;
    double accel_x, accel_y, accel_z;
};

struct JointSample {
    double timestamp;
    std::vector<std::string> names;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
};


class RobotSimulatort : public rclcpp::Node
{
public:
    RobotSimulatort()
    : Node("robot_simulator"), count_(0)
    {
        
        odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/tita4267305/chassis/odometry", 10);
        timer_odom_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotSimulatort::publish_odom, this));

        wheeled_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/perception/wheel/tracking/Odometry", 10);
        timer_wheeled_odom_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotSimulatort::publish_wheel_odom, this));

        imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/tita4267305/imu_sensor_broadcaster/imu", 10);
        timer_imu_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&RobotSimulatort::publish_imu, this));

        joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/tita4267305/joint_states", 10);
        timer_joint_state_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&RobotSimulatort::publish_joint_state, this));


        // Load IMU data from file
        if (!load_imu_data("/home/emiliano/Desktop/ros2_ws/src/robot_data_giu_su/imu_log.txt")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load imu.txt");
        }
        imu_index_ = 0;

        // Load joint data from file
        if (!load_joint_data("/home/emiliano/Desktop/ros2_ws/src/robot_data_giu_su/joint_state_log.txt")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load joint_state.txt");
        }
        joint_index_ = 0;
    }

private:

        
       
        bool load_joint_data(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open joint state file");
            return false;
        }

        std::string line;
        double current_timestamp = -1.0;
        JointSample sample;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '-') continue;  // skip separators

            std::stringstream ss(line);
            double timestamp;
            std::string joint_name_colon, tmp;
            double pos, vel, effort;
            std::string joint_name;   // declare here so it's in scope outside try

            try {
                ss >> timestamp;               // read timestamp
                ss >> joint_name_colon;        // joint name with colon
                joint_name = joint_name_colon;
                if (!joint_name.empty() && joint_name.back() == ':')
                    joint_name.pop_back();     // remove trailing colon

                ss >> tmp >> pos;              // pos: value
                ss >> tmp >> vel;              // vel: value
                ss >> tmp >> effort;           // effort: value

            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid line: %s", line.c_str());
                continue;
            }

            // Check if we reached a new timestamp
            if (timestamp != current_timestamp) {
                if (current_timestamp >= 0 && !sample.names.empty()) {
                    joint_samples_.push_back(sample); // save previous sample
                }
                sample = JointSample();               // start new sample
                sample.timestamp = timestamp;
                current_timestamp = timestamp;
            }

            // Add joint data to current sample
            sample.names.push_back(joint_name);
            sample.positions.push_back(pos);
            sample.velocities.push_back(vel);
            sample.efforts.push_back(effort);

            // Debug print each joint
            RCLCPP_INFO(this->get_logger(),
                "Timestamp: %.6f Joint: %s Pos: %.6f Vel: %.6f Effort: %.6f",
                timestamp, joint_name.c_str(), pos, vel, effort);
        }

        // Push the last sample
        if (!sample.names.empty()) {
            joint_samples_.push_back(sample);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu joint samples", joint_samples_.size());
        return true;
    }



        bool load_imu_data(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open IMU file");
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;

            ImuData imu;
            std::stringstream ss(line);
            std::string label;
            double timestamp;

            try {
                // Timestamp
                ss >> label >> timestamp;

                // Orientation
                ss >> label; // "orientation:"
                ss >> imu.orientation_x >> imu.orientation_y >> imu.orientation_z >> imu.orientation_w;

                // Angular velocity
                ss >> label; // "angular_velocity:"
                ss >> imu.angular_x >> imu.angular_y >> imu.angular_z;

                // Linear acceleration
                ss >> label; // "linear_acceleration:"
                ss >> imu.accel_x >> imu.accel_y >> imu.accel_z;

                RCLCPP_INFO(this->get_logger(),
                "IMU: orient=(%.6f, %.6f, %.6f, %.6f) ang_vel=(%.6f, %.6f, %.6f) accel=(%.6f, %.6f, %.6f)",
                imu.orientation_x, imu.orientation_y, imu.orientation_z, imu.orientation_w,
                imu.angular_x, imu.angular_y, imu.angular_z,
                imu.accel_x, imu.accel_y, imu.accel_z);

            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid line: %s", line.c_str());
                continue;
            }

            imu_data_.push_back(imu);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu IMU lines", imu_data_.size());
                return true;
    }



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
        odom_msg.pose.pose.orientation.x = get_random(0.0, 5.0);
        odom_msg.pose.pose.orientation.y = get_random(0.0, 5.0);
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
        if (imu_data_.empty()) return;

        auto& imu_sample = imu_data_[imu_index_];

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        msg.orientation.x = imu_sample.orientation_x;
        msg.orientation.y = imu_sample.orientation_y;
        msg.orientation.z = imu_sample.orientation_z;
        msg.orientation.w = imu_sample.orientation_w;

        msg.angular_velocity.x = imu_sample.angular_x;
        msg.angular_velocity.y = imu_sample.angular_y;
        msg.angular_velocity.z = imu_sample.angular_z;

        msg.linear_acceleration.x = imu_sample.accel_x;
        msg.linear_acceleration.y = imu_sample.accel_y;
        msg.linear_acceleration.z = imu_sample.accel_z;

        imu_->publish(msg);

        // Go to next sample, loop if needed
        imu_index_ = (imu_index_ + 1) % imu_data_.size();

        // auto msg = sensor_msgs::msg::Imu();
        // msg.header.stamp = this->now();
        // msg.header.frame_id = "imu_link";

        // // Orientation (no rotation)
        // msg.orientation.x = get_random(0.0, 1.0);
        // msg.orientation.y = get_random(0.0, 1.0);
        // msg.orientation.z = get_random(0.0, 1.0);
        // msg.orientation.w = get_random(0.0, 1.0);

        // // Angular velocity (gyro)
        // msg.angular_velocity.x = get_random(-0.1, 0.1);
        // msg.angular_velocity.y = get_random(-0.1, 0.1);
        // msg.angular_velocity.z = get_random(-0.2, 0.2);

        // // Linear acceleration (accelerometer)
        // msg.linear_acceleration.x = get_random(-0.2, 0.2);
        // msg.linear_acceleration.y = get_random(-0.2, 0.2);
        // msg.linear_acceleration.z = 9.8 + get_random(-0.1, 0.1);  // Gravity plus noise

        // imu_->publish(msg);
    }



    void publish_joint_state()
    {
        if (joint_samples_.empty()) return;

        auto &s = joint_samples_[joint_index_];

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = s.names;
        msg.position = s.positions;
        msg.velocity = s.velocities;
        msg.effort = s.efforts;

        joint_state_->publish(msg);

        joint_index_ = (joint_index_ + 1) % joint_samples_.size();

        // auto msg = sensor_msgs::msg::JointState();

        // // Fill the header
        // msg.header.stamp = this->now();
        // msg.header.frame_id = "";

        // // Define joint names
        // msg.name = {
        //     "joint_left_leg_1", "joint_left_leg_2", "joint_left_leg_3", "joint_left_leg_4",
        //     "joint_right_leg_1", "joint_right_leg_2", "joint_right_leg_3", "joint_right_leg_4"
        // };

        // // Generate random positions and velocities
        // msg.position.resize(msg.name.size());
        // msg.velocity.resize(msg.name.size());
        // msg.effort.resize(msg.name.size()); // Optional: fill with zeros

        // for (size_t i = 0; i < msg.name.size(); ++i)
        // {
        //     msg.position[i] = 3.14;   // Simulate joint angles (in radians)
        //     msg.velocity[i] = 1.19;     // Simulate velocity
        //     msg.effort[i] = 0.0;                         // No effort simulation here
        // }

        // joint_state_->publish(msg);
    }


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
    rclcpp::TimerBase::SharedPtr timer_odom_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheeled_odom_;
    rclcpp::TimerBase::SharedPtr timer_wheeled_odom_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::TimerBase::SharedPtr timer_imu_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
    rclcpp::TimerBase::SharedPtr timer_joint_state_;

    
    std::vector<JointSample> joint_samples_;
    size_t joint_index_;
    std::vector<ImuData> imu_data_;
    size_t imu_index_;

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
