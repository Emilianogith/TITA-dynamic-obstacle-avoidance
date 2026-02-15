#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "std_msgs/msg/bool.hpp"


#include <fstream>
#include <iomanip> 
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <WalkingManager.hpp>



inline double wrapToPi(double q)
{
    return std::atan2(std::sin(q), std::cos(q));
}


struct RobotSensors{

    // imu sensor
    struct ImuSensor{
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_acceleration = Eigen::Vector3d(0,0,9.81); 
    };

    // odom floating base translation and body orientation
    struct Odom{
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    };

    // joint states
    struct JointState{
        double pos = 0.0;
        double vel = 0.0;
    };

    std::unordered_map<std::string, JointState> joints;

    ImuSensor imu;
    Odom odom;
};



class RobotController : public rclcpp::Node
{
public:
    RobotController()
    : Node("robot_controller")
    {
        // Subscriber for /joint_states
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/tita4267305/joint_states", 1,
            std::bind(&RobotController::joint_states_callback, this, std::placeholders::_1));

         // Subscriber for /security_stop
        security_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/security_stop", 1,
            std::bind(&RobotController::security_stop_callback, this, std::placeholders::_1));
    
    
        rclcpp::QoS cmd_qos(rclcpp::KeepLast(10));
        cmd_qos.best_effort();

        position_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_position/commands", cmd_qos);
        velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_velocity/commands", cmd_qos);
        effort_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_effort/commands", cmd_qos);
        kp_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_kp/commands", cmd_qos);
        kd_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_kd/commands", cmd_qos);
        timer_effort_cmd_ = this->create_wall_timer(
            std::chrono::milliseconds(2),   // controller at 500 Hz
            std::bind(&RobotController::publish_joint_command, this));



        // ------------------ Build Pinocchio model ------------------
        std::string robot_description_filename = "/home/emiliano/Desktop/ros2_ws/src/tita_controller/tita_description/tita.urdf";

        pinocchio::Model full_robot_model;
        pinocchio::JointModelFreeFlyer root_joint;
        pinocchio::urdf::buildModel(robot_description_filename, root_joint, full_robot_model);
        // lock joints if you want (empty now)
        const std::vector<std::string> joint_to_lock_names{};
        std::vector<pinocchio::JointIndex> joint_ids_to_lock;
        for (const auto& joint_name : joint_to_lock_names)
        {
            if (full_robot_model.existJointName(joint_name))
            joint_ids_to_lock.push_back(full_robot_model.getJointId(joint_name));
        }
        robot_model_ = pinocchio::buildReducedModel(
            full_robot_model,
            joint_ids_to_lock,
            pinocchio::neutral(full_robot_model));

        robot_data_ = pinocchio::Data(robot_model_);
    
        

        // open log files
        joint_state_log_.open("joint_state_log.txt");
    }



private:

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
            {
                auto &joint = robot_sensor_.joints[msg->name[i]];  // creates if missing
                joint.pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
                joint.vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0; 

                double effort = i < msg->effort.size() ? msg->effort[i] : 0.0;

                // logs the joint values
                joint_state_log_ <<std::fixed << std::setprecision(9) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " "
                << msg->name[i] << ":  pos: "
                << joint.pos << " vel: "
                << joint.vel << " effort: "
                << effort << std::endl;
            }

        joint_state_log_ << "----------------------------------------" << std::endl;

    }

    void publishFixedGains()
    {
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray kp_msg, kd_msg;

        kp_msg.data.resize(na, 0.0);            // 0.5
        kd_msg.data.resize(na, 0.0);            // 0.2

        // ---------------- only for regulation -------------------------
        kp_msg.data[6] = 100.0;       // 100      // right leg 3
        kd_msg.data[6] = 0.8;       // 0.8

        // kp_msg.data[5] = 1.0;   // right leg 3           // TODO: tuna questo guadagno
        // kd_msg.data[5] = 0.8;

        // kp_msg.data[2] = 80.0;   // left leg 3
        // kd_msg.data[2] = 0.5;

        kp_msg.data[3] = 0.0;
        kp_msg.data[7] = 0.0;       // 5.0
        kd_msg.data[3] = 0.0;
        kd_msg.data[7] = 0.0;       // 0.2
        // --------------------------------------------------------------

        kp_cmd_pub_->publish(kp_msg);           // publish kp only once for tita_bridge control
        kd_cmd_pub_->publish(kd_msg);           // publish kd only once for tita_bridge control
    }

    bool fillMsgs(std_msgs::msg::Float64MultiArray& position_msg, 
        std_msgs::msg::Float64MultiArray& velocity_msg, 
        std_msgs::msg::Float64MultiArray& effort_msg,
        const double& t_sec)
    {
        size_t idx = 0;
        for (pinocchio::JointIndex j = 2; static_cast<int>(j) < robot_model_.njoints; ++j, ++idx) {

            // ---------------- only for regulation -------------------------
            effort_msg.data[idx]   = 0.0;
            velocity_msg.data[idx] = 0.0;
        }


        const double v_leg4 = -0.05;
        double q_cmd = -0.7 + v_leg4 * t_sec;
        q_cmd = std::clamp(q_cmd, -0.9, -0.7);

        const double v_r_wheel = 0.0;           // TODO: testa comando in velocita

        if (t_sec < 5){
            position_msg.data[0] = -0.6;
            position_msg.data[1] = 0.0;
            position_msg.data[2] = -0.8;
            position_msg.data[3] = 2.4;         // left wheel
            position_msg.data[4] = 0.50;
            
            // right leg 2
            position_msg.data[5] = 0.0;
            // velocity_msg.data[5] = 0.0;

            // right leg 3
            position_msg.data[6] = q_cmd;     // q_cmd;
            velocity_msg.data[6] = v_leg4;

            // right wheel 
            position_msg.data[7] = 0.5;         // right wheel
            velocity_msg.data[7] = v_r_wheel;           // vel right wheel      : ruote controllate in velocita
        }   else {
            position_msg.data[0] = -0.6;
            position_msg.data[1] = 0.0;
            position_msg.data[2] = -0.9;
            position_msg.data[3] = 2.2;                // left wheel             : ruote controllate in velocita
            position_msg.data[4] = 0.50;
            
            // right leg 2
            position_msg.data[5] = 0.0;
            // velocity_msg.data[5] = 0.0;

            // right leg 3
            position_msg.data[6] = -0.9;
            velocity_msg.data[6] = 0.0;

            // right wheel
            position_msg.data[7] = 0.4;        // right wheel
            velocity_msg.data[7] = 0.0;         // vel right wheel
        }



        // ---- Limit checking ----
        const size_t na = robot_model_.njoints - 2;
        const auto& q_min = robot_model_.lowerPositionLimit.tail(na);
        const auto& q_max = robot_model_.upperPositionLimit.tail(na);

        for (size_t i = 0; i < na; ++i)
        {
            if (position_msg.data[i] < q_min[i] ||
                position_msg.data[i] > q_max[i])
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Position command for joint %zu out of limits! [%f, %f], cmd=%f",
                    i + 1, q_min[i], q_max[i], position_msg.data[i]
                );
                return false;
            }
        }
        return true;
    }

    void sendZeroCommand()
    {
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray position_msg, velocity_msg, effort_msg, kp_msg, kd_msg;
        // position_msg.data.resize(na, 0.0);
        velocity_msg.data.resize(na, 0.0);
        effort_msg.data.resize(na, 0.0);
        kp_msg.data.resize(na, 0.0);
        kd_msg.data.resize(na, 0.0);


        kp_cmd_pub_->publish(kp_msg); 
        kd_cmd_pub_->publish(kd_msg);
        effort_cmd_pub_->publish(effort_msg);
        velocity_cmd_pub_->publish(velocity_msg);
        // position_cmd_pub_->publish(position_msg);
    }

    void publish_joint_command()
    {
        if (init_time){
            init_time = false;
            start_time_ = this->now();
        }

        const rclcpp::Time t_now = this->now();
        const double t_from_start= (t_now - start_time_).seconds();
        if (t_from_start < time_offset_) return;

        const double t_sec = t_from_start - time_offset_;



        if (first_callback_){
            first_callback_ = false;
            // publishFixedGains();         // non lo attua se lo mandi una volta sola 
        }
       
        RCLCPP_INFO(this->get_logger(), "t_sec: %f", t_sec);


        // ---------- Messages fill ----------
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray position_msg, velocity_msg, effort_msg;
        position_msg.data.resize(na, 0.0);
        velocity_msg.data.resize(na, 0.0);
        effort_msg.data.resize(na, 0.0);
    

        if (!fillMsgs(position_msg, velocity_msg, effort_msg, t_sec) || security_stop_) {
            if (security_stop_)
                RCLCPP_WARN(this->get_logger(), "Security stop activated! Sending zero commands.");
            sendZeroCommand();
            return;
        }

        publishFixedGains();
      
        // ---------- Publish ----------
        position_cmd_pub_->publish(position_msg);
        velocity_cmd_pub_->publish(velocity_msg);
        effort_cmd_pub_->publish(effort_msg);
    }

    void security_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        security_stop_ = msg->data;
        if (security_stop_) {
            RCLCPP_WARN(this->get_logger(), "Security stop activated! Sending zero commands.");
            sendZeroCommand();
        }
    }



    // initialize subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr security_stop_sub_;
    bool security_stop_ = false;

    // initialize publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kp_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kd_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_effort_cmd_;

    
    RobotSensors robot_sensor_;

    rclcpp::Time start_time_;
    bool init_time = true;
    double time_offset_ = 5;   
    bool first_callback_ = true;
    double nominal_dt_ = 0.002;                 // controller nomianal period

    pinocchio::Model robot_model_;
    pinocchio::Data robot_data_;      

    // log files
    std::ofstream joint_state_log_;
    std::ofstream joint_eff_log_file_;
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
