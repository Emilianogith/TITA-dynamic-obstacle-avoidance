#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "std_msgs/msg/bool.hpp"

#include "std_srvs/srv/trigger.hpp"

#include <fstream>
#include <iomanip> 
#include <filesystem>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <StateFilter_no_bias.hpp>
#include <RobotOdometry.hpp>
#include <WalkingManager.hpp>



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

    struct WheelState{
        double pos_prev = 0.0;
        rclcpp::Time t_prev{0, 0, RCL_ROS_TIME};
        double alpha = 1.0;                         // low-pass filter parameter
    };

    std::unordered_map<std::string, JointState> joints;

    ImuSensor imu;
    Odom odom;
    WheelState wheel_left;
    WheelState wheel_right;
};



class RobotController : public rclcpp::Node
{
public:
    RobotController()
    : Node("robot_controller")
    {
        // Subscriber for /chassis/odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/tita4267305/chassis/odometry", 1,
            std::bind(&RobotController::odom_callback, this, std::placeholders::_1));

        // Subscriber for /imu_sensor_broadcaster/imu
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/tita4267305/imu_sensor_broadcaster/imu", 1,
            std::bind(&RobotController::imu_callback, this, std::placeholders::_1));

        // Subscriber for /joint_states
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/tita4267305/joint_states", 1,
            std::bind(&RobotController::joint_states_callback, this, std::placeholders::_1));


        // Publisher for /filtered_state
        filtered_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/filtered_state", 10);
        timer_filtered_state_ = this->create_wall_timer(
            std::chrono::milliseconds(2),   // filter at 500 Hz
            std::bind(&RobotController::publish_filtered_state, this));
            
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publisher for /tita_hw/effort_controller/commands
        effort_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/effort_controller/commands", 10);
        timer_effort_cmd_ = this->create_wall_timer(
            std::chrono::milliseconds(2),   // controller at 500 Hz
            std::bind(&RobotController::publish_joint_command, this));


        // Services for controller modalities
        security_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "security_stop",
            std::bind(&RobotController::securityStopService, this,
                    std::placeholders::_1, std::placeholders::_2));

        regulation_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "regulation_mode",
            std::bind(&RobotController::regulationModeService, this,
                    std::placeholders::_1, std::placeholders::_2));

        start_filter_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_filter",
            std::bind(&RobotController::startFilterService, this,
                    std::placeholders::_1, std::placeholders::_2));

        whole_body_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "whole_body_mode",
            std::bind(&RobotController::wholeBodyModeService, this,
                    std::placeholders::_1, std::placeholders::_2));


        
        // ------------------ Build Pinocchio model ------------------
        std::string robot_description_filename = std::string(std::getenv("HOME")) + "/Desktop/ros2_ws/src/tita_controller/tita_description/tita.urdf";

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
        right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
        left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
    


        state_filter_ptr_ = std::make_shared<labrob::KF>(robot_model_);        
        robot_odometry_ptr_ = std::make_shared<labrob::RobotOdometry>(robot_model_);



        // Logging
        std::string prefix = std::string(std::getenv("HOME")) + "/Desktop/ros2_ws/robot_logs/";

        // Create directory if it doesn't exist
        std::filesystem::create_directories(prefix);

        // ---------- KF log ----------
        csv.open(prefix + "kf_test.csv");
        csv << "t,"
            << "p_odom_x,p_odom_y,p_odom_z,"
            << "p_est_x,p_est_y,p_est_z,"
            << "v_est_x,v_est_y,v_est_z,"
            << "p_cL_est_x,p_cL_est_y,p_cL_est_z,"
            << "p_cR_est_x,p_cR_est_y,p_cR_est_z\n";

        // ---------- Robot odometry log ----------
        robot_odom_log.open(prefix + "robot_odom.csv");
        robot_odom_log << "t,"
            << "p_odom_x,p_odom_y,p_odom_z,"
            << "v_odom_x,v_odom_y,v_odom_z,"
            << "p_cL_odom_x,p_cL_odom_y,p_cL_odom_z,"
            << "p_cR_odom_x,p_cR_odom_y,p_cR_odom_z,"
            << "w_l_odom,w_r_odom\n";

        // ---------- Wheel log ----------
        wheel_log_.open(prefix + "wheel_log.txt");
        wheel_log_ << "Timestamp, Joint Name, Position, Velocity, Velocity Difference\n";

        // ---------- Other logs ----------
        odom_log_.open(prefix + "odom.txt");
        imu_log_.open(prefix + "imu_log.txt");
        joint_state_log_.open(prefix + "joint_state_log.txt");

        // ---------- Joint effort log ----------
        joint_eff_log_file_.open(prefix + "joint_eff.txt");
    }



private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_sensor_.odom.position = Eigen::Vector3d(
                                                msg->pose.pose.position.x,
                                                msg->pose.pose.position.y,
                                                msg->pose.pose.position.z
                                            );

        robot_sensor_.odom.orientation = Eigen::Quaterniond(
                                                msg->pose.pose.orientation.w,
                                                msg->pose.pose.orientation.x,
                                                msg->pose.pose.orientation.y,
                                                msg->pose.pose.orientation.z
                                            );
        robot_sensor_.odom.orientation.normalize();



        odom_log_ 
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

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        robot_sensor_.imu.orientation = Eigen::Quaterniond(
                                                msg->orientation.w,
                                                msg->orientation.x,
                                                msg->orientation.y,
                                                msg->orientation.z
                                            );
        robot_sensor_.imu.orientation.normalize();

        robot_sensor_.imu.angular_velocity = Eigen::Vector3d(
                                                msg->angular_velocity.x,
                                                msg->angular_velocity.y,
                                                msg->angular_velocity.z
                                            );

        robot_sensor_.imu.linear_acceleration = Eigen::Vector3d(
                                                msg->linear_acceleration.x,
                                                msg->linear_acceleration.y,
                                                msg->linear_acceleration.z
                                            );

        imu_log_ << "Timestamp: " << std::fixed << std::setprecision(9) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " "
            << "orientation: " << msg->orientation.x << " " << msg->orientation.y << " "
            << msg->orientation.z << " " << msg->orientation.w << " "
            << "angular_velocity: " << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << " "
            << "linear_acceleration: " << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z
            << std::endl;
    }

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

        // fill joint state
        for(pinocchio::JointIndex joint_id = 2; static_cast<int>(joint_id) < robot_model_.njoints; ++joint_id){
            const std::string& name = robot_model_.names[joint_id];                                 // get joint name from Pinocchio
            robot_state_.joint_state[name].pos = robot_sensor_.joints[name].pos; 
            robot_state_.joint_state[name].vel = robot_sensor_.joints[name].vel; 

            // special handling for wheel joints to apply low-pass filter and log
            if (name == "joint_left_leg_4"){

                double alpha = robot_sensor_.wheel_left.alpha;
                double pos_prev = robot_sensor_.wheel_left.pos_prev;

                double filtered_pos = alpha * robot_state_.joint_state[name].pos + (1 - alpha) * pos_prev;

                rclcpp::Time t = msg->header.stamp;
                double dt = (t - robot_sensor_.wheel_left.t_prev).seconds();  

                wheel_log_ <<std::fixed << std::setprecision(9) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " "
                << name << 
                ":  pos: " << robot_state_.joint_state[name].pos <<
                ":  filtered pos: " << filtered_pos <<
                " vel: " << robot_state_.joint_state[name].vel  <<
                " vel_diff: " << (filtered_pos - robot_sensor_.wheel_left.pos_prev) / dt  << std::endl;

                robot_sensor_.wheel_left.pos_prev = filtered_pos;
                robot_sensor_.wheel_left.t_prev = t;
            }
            else if (name == "joint_right_leg_4"){
                
                double alpha = robot_sensor_.wheel_right.alpha;
                double pos_prev = robot_sensor_.wheel_right.pos_prev;

                double filtered_pos = alpha * robot_state_.joint_state[name].pos + (1 - alpha) * pos_prev;

                rclcpp::Time t = msg->header.stamp;
                double dt = (t - robot_sensor_.wheel_right.t_prev).seconds();  
                
                wheel_log_ <<std::fixed << std::setprecision(9) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " "
                << name << 
                ":  pos: " << robot_state_.joint_state[name].pos <<
                ":  filtered pos: " << filtered_pos <<
                " vel: " << robot_state_.joint_state[name].vel  <<
                " vel_diff: " << (filtered_pos - robot_sensor_.wheel_right.pos_prev) / dt  << std::endl;
                
                robot_sensor_.wheel_right.pos_prev = filtered_pos;
                robot_sensor_.wheel_right.t_prev = t;
            }

        }

        // needed to ensure joint states are received at least once before starting the controller
        if (!received_joint_state_) received_joint_state_ = true;
    }

    void publish_filtered_state()
    {
        if(!start_filter_) return;                                                                          // wait for the service to start the filter

        // ---------- fill filter parameters ----------
        Eigen::Vector<double, 12> filter_params;
        filter_params.setZero();
        filter_params.segment<4>(0) = robot_sensor_.imu.orientation.coeffs();
        
        Eigen::Vector<double, 14> filter_input;
        filter_input.setZero();
        filter_input.segment<3>(0) = robot_sensor_.imu.linear_acceleration;
        filter_input.segment<3>(3) = robot_sensor_.imu.angular_velocity;

        // Manual calibration: add some offset to IMU measurements to compensate for the bias 
        filter_input(3) += -0.0017;
        filter_input(4) += 0.0036;
        filter_input(5) += 0.000230;

        for(pinocchio::JointIndex joint_id = 2; static_cast<int>(joint_id) < robot_model_.njoints; ++joint_id){
            const std::string& name = robot_model_.names[joint_id];                                         // get joint name from Pinocchio
            filter_params(4 + joint_id - 2) = robot_sensor_.joints[name].pos;                               // Joint positions
            filter_input(6 + joint_id - 2) = robot_sensor_.joints[name].vel;                                // Joint velocities
        }

        // Manual calibration: add some offset to wheel velocity measurements to compensate for the bias
        filter_input(6+3) += 0.005;
        filter_input(6+7) -= 0.005;
        // -------------------------------------------
        

        rclcpp::Time t_now = this->now();
        if (!initialized_filter) {
            start_time_filter_ = t_now;                                                                     // store filter initial time
            initialized_filter = true;
            t_prev_ = start_time_filter_;

            // ------------------ Filter init ----------------------------
            state_filter_ptr_->set_initial_condition(filter_params.segment<8>(4), robot_sensor_.imu.orientation);
            // ------------------ Reset odometry -------------------------
            robot_odometry_ptr_->reset(filter_params.segment<8>(4), robot_sensor_.imu.orientation);        
        }

        double dt = (t_now - t_prev_).seconds();                                                            // compute effective dt
        t_prev_ = t_now;
        
        // --------- Run filter ---------
        bool in_contact = true;
        Eigen::Vector<double,12> filtered_state = state_filter_ptr_->compute_KF_estimate(filter_input, filter_params, dt, in_contact);

        
        // ---- Log ---------------------
        double t_rel = (t_now - start_time_filter_).seconds();                                               // Compute relative time
        csv << std::fixed << std::setprecision(9);
        csv << t_rel << ","
            << robot_sensor_.odom.position(0) << "," << robot_sensor_.odom.position(1) << "," << robot_sensor_.odom.position(2) << ","
            << filtered_state(0) << "," << filtered_state(1) << "," << filtered_state(2) << ","
            << filtered_state(3) << "," << filtered_state(4) << "," << filtered_state(5) << ","
            << filtered_state(6) << "," << filtered_state(7) << "," << filtered_state(8) << ","
            << filtered_state(9) << "," << filtered_state(10) << "," << filtered_state(11) << "\n";


        // ------ fill robot state -------
        robot_state_.position = filtered_state.segment<3>(0);
        robot_state_.orientation.coeffs() = filter_params.segment<4>(0);
        robot_state_.linear_velocity = robot_state_.orientation.toRotationMatrix().transpose() * filtered_state.segment<3>(3);
        robot_state_.angular_velocity = filter_input.segment<3>(3);


        // --------- Compute odometry ---------
        labrob::Odom robot_odom = robot_odometry_ptr_->forward_step(filter_params.segment<8>(4), 
        filter_input.segment<8>(6), 
        robot_sensor_.imu.orientation,
        filter_input.segment<3>(3),
        dt);

        robot_odom_log << std::fixed << std::setprecision(9);
        robot_odom_log << t_rel << ","
            << robot_odom.position(0) << "," << robot_odom.position(1) << "," << robot_odom.position(2) << ","
            << robot_odom.velocity(0) << "," << robot_odom.velocity(1) << "," << robot_odom.velocity(2) << ","
            << robot_odom.pc_l(0) << "," << robot_odom.pc_l(1) << "," << robot_odom.pc_l(2) << ","
            << robot_odom.pc_r(0) << "," << robot_odom.pc_r(1) << "," << robot_odom.pc_r(2) << ","
            << robot_odom.w_l << "," << robot_odom.w_r << "\n";


        // --------- Publish filtered state ---------
        // Publish odometry message
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base";

        msg.pose.pose.position.x = robot_state_.position.x();
        msg.pose.pose.position.y = robot_state_.position.y();
        msg.pose.pose.position.z = robot_state_.position.z();

        msg.pose.pose.orientation.x = robot_state_.orientation.x();
        msg.pose.pose.orientation.y = robot_state_.orientation.y();
        msg.pose.pose.orientation.z = robot_state_.orientation.z();
        msg.pose.pose.orientation.w = robot_state_.orientation.w();

        msg.twist.twist.linear.x = robot_state_.linear_velocity.x();
        msg.twist.twist.linear.y = robot_state_.linear_velocity.y();
        msg.twist.twist.linear.z = robot_state_.linear_velocity.z();

        msg.twist.twist.angular.x = robot_state_.angular_velocity.x();
        msg.twist.twist.angular.y = robot_state_.angular_velocity.y();
        msg.twist.twist.angular.z = robot_state_.angular_velocity.z();
        filtered_state_pub_->publish(msg);


        // Publish transforms
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = this->now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base";
        odom_tf.transform.translation.x = robot_state_.position.x();
        odom_tf.transform.translation.y = robot_state_.position.y();
        odom_tf.transform.translation.z = robot_state_.position.z();

        odom_tf.transform.rotation.x = robot_state_.orientation.x();
        odom_tf.transform.rotation.y = robot_state_.orientation.y();
        odom_tf.transform.rotation.z = robot_state_.orientation.z();
        odom_tf.transform.rotation.w = robot_state_.orientation.w();
        tf_broadcaster_->sendTransform(odom_tf);



        // Publish contact frames
        Eigen::Vector<double, 8> q_joint = filter_params.tail<8>();
        Eigen::Quaterniond q_base;
        q_base.coeffs() << filter_params(0), filter_params(1), filter_params(2), filter_params(3);          // (x,y,z,w)
        q_base.normalize();
        Eigen::Vector<double, 3 + 4 + 8> q;
        q << Eigen::Vector3d(0,0,0), 
        q_base.coeffs(),
        q_joint;
        
        // ------> Compute contact frames orientation 
        pinocchio::framesForwardKinematics(robot_model_, robot_data_, q);
        pinocchio::computeJointJacobians(robot_model_, robot_data_, q); 

        Eigen::Matrix3d r_wheel_R = robot_data_.oMf[right_leg4_idx_].rotation();
        Eigen::Matrix3d l_wheel_R = robot_data_.oMf[left_leg4_idx_].rotation();
        Eigen::Matrix3d r_contact_frame_R = labrob::compute_contact_frame(r_wheel_R);
        Eigen::Matrix3d l_contact_frame_R = labrob::compute_contact_frame(l_wheel_R);

        Eigen::Quaterniond q_p_cR(r_contact_frame_R);
        q_p_cR.normalize();

        Eigen::Quaterniond q_p_cL(l_contact_frame_R);
        q_p_cL.normalize();


        geometry_msgs::msg::TransformStamped p_cL_tf;
        p_cL_tf.header.stamp = this->now();
        p_cL_tf.header.frame_id = "odom";
        p_cL_tf.child_frame_id = "p_cL";

        p_cL_tf.transform.translation.x = filtered_state(6);
        p_cL_tf.transform.translation.y = filtered_state(7);
        p_cL_tf.transform.translation.z = filtered_state(8);

        p_cL_tf.transform.rotation.x = q_p_cL.x();                                                          // <------
        p_cL_tf.transform.rotation.y = q_p_cL.y();                                                          // <------
        p_cL_tf.transform.rotation.z = q_p_cL.z();                                                          // <------
        p_cL_tf.transform.rotation.w = q_p_cL.w();                                                          // <------
        tf_broadcaster_->sendTransform(p_cL_tf);

        geometry_msgs::msg::TransformStamped p_cR_tf;
        p_cR_tf.header.stamp = this->now();
        p_cR_tf.header.frame_id = "odom";
        p_cR_tf.child_frame_id = "p_cR";

        p_cR_tf.transform.translation.x = filtered_state(9);
        p_cR_tf.transform.translation.y = filtered_state(10);
        p_cR_tf.transform.translation.z = filtered_state(11);

        p_cR_tf.transform.rotation.x = q_p_cR.x();                                                          // <------
        p_cR_tf.transform.rotation.y = q_p_cR.y();                                                          // <------                        
        p_cR_tf.transform.rotation.z = q_p_cR.z();                                                          // <------              
        p_cR_tf.transform.rotation.w = q_p_cR.w();                                                          // <------                
        tf_broadcaster_->sendTransform(p_cR_tf);
    }

    void sendZeroCommand()
    {
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray effort_msg;

        effort_msg.data.resize(na, 0.0);

        effort_cmd_pub_->publish(effort_msg);
    }

    void initWalkingManagerOnce()
    {
        // TODO: armatures 
        std::map<std::string, double> armatures;
        armatures.clear();

        // Walking Manager:
        walking_manager_.init(robot_state_, armatures, robot_model_);
        initialized_walking_manager_ = true;
    }

    void regulate_robot(std::array<double, 8> KP,
        std::array<double, 8> KD,
        std_msgs::msg::Float64MultiArray& effort_msg){
        /*Joint-level PD regulation towards a nominal configuration,
         used for testing the controller and for safety when the robot is lifted off the ground*/

        std::array<double, 8> q_des;
        q_des[0] = 0.0;                 // joint_left_leg_1
        q_des[1] = 0.5;                 // joint_left_leg_2
        q_des[2] = -1.0;                // joint_left_leg_3
        q_des[3] = 0.1;                 // joint_left_leg_4
        q_des[4] = 0.0;                 // joint_right_leg_1
        q_des[5] = 0.5;                 // joint_right_leg_2
        q_des[6] = -1.0;                // joint_right_leg_3
        q_des[7] = 0.1;                 // joint_right_leg_4

        std::array<double, 8> qdot_des;
        qdot_des[0] = 0.0;              // joint_left_leg_1
        qdot_des[1] = 0.0;              // joint_left_leg_2
        qdot_des[2] = 0.0;              // joint_left_leg_3
        qdot_des[3] = 0.0;              // joint_left_leg_4
        qdot_des[4] = 0.0;              // joint_right_leg_1
        qdot_des[5] = 0.0;              // joint_right_leg_2
        qdot_des[6] = 0.0;              // joint_right_leg_3
        qdot_des[7] = 0.0;              // joint_right_leg_4

        size_t idx = 0;
        for (pinocchio::JointIndex j = 2; static_cast<int>(j) < robot_model_.njoints; ++j, ++idx) {
            const auto& name = robot_model_.names[j];
            // get current joint state 
            const auto& js_curr = robot_state_.joint_state[name];      
            const double vel_curr = js_curr.vel;
            const double pos_curr = js_curr.pos;
            effort_msg.data[idx] = KP[idx] * labrob::angleError(q_des[idx], pos_curr) + KD[idx] * (qdot_des[idx] - vel_curr);
        }
    }

    bool fillMsgs(
        const labrob::JointCommand& tau,
        const labrob::JointCommand& qdd,
        std::array<double, 8> KP,
        std::array<double, 8> KD,
        std_msgs::msg::Float64MultiArray& effort_msg)
    {

        const size_t na = robot_model_.njoints - 2;
        const auto& q_min = robot_model_.lowerPositionLimit.tail(na);
        const auto& q_max = robot_model_.upperPositionLimit.tail(na);

        size_t idx = 0;
        for (pinocchio::JointIndex j = 2; static_cast<int>(j) < robot_model_.njoints; ++j, ++idx) {
            const auto& name = robot_model_.names[j];
            const double u = tau[name];
            const double a = qdd[name];

            // check if WBC returned NAN
            if (!std::isfinite(a)) {
                RCLCPP_ERROR(get_logger(), "NaN/Inf command on joint %s", name.c_str());
                return false;
            }

            const auto& js = robot_state_.joint_state[name];      
            double vel_des = js.vel + a * nominal_dt_;
            double pos_des = js.pos + js.vel * nominal_dt_ + 0.5 * a * nominal_dt_ * nominal_dt_;

            // check if computed accelerations respect joint limits
            if (pos_des < q_min[idx] || pos_des > q_max[idx])
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Position command for joint %zu out of limits! [%f, %f], cmd=%f",
                    idx + 1, q_min[idx], q_max[idx], pos_des
                );
                return false;
            }

            // -------------- fill the message with FF + PD low-level control law ---------------
            effort_msg.data[idx] = u + KP[idx] * labrob::angleError(pos_des, js.pos) + KD[idx] * (vel_des - js.vel);
    
            // effort_msg.data[idx] = KP[idx] * labrob::angleError(pos_des, js.pos) + KD[idx] * (vel_des - js.vel);
        }
        return true;
    }

    void publish_joint_command()
    {
        /*Publishes the joint command at each control step, depending on the selected control mode*/

        if (security_stop_){
            RCLCPP_WARN(this->get_logger(), "Security stop activated! Sending zero commands.");
            sendZeroCommand();
            return;
        }


        // ------------ Messages initialization ------------
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(na, 0.0);

        // low-level gains
        std::array<double, 8> KP = {                                            // std::array<double, 8> KP = {
                10.0, 10.0, 10.0,  0.8,                                         //         40.0, 40.0, 40.0,  0.8,
                10.0, 10.0, 10.0,  0.8                                          //         40.0, 40.0, 40.0,  0.8
            };                                                                  //     };
                                            
        std::array<double, 8> KD = {                                            // std::array<double, 8> KD = {
                0.2, 0.2, 0.2,  0.1,                                            //         1.5, 1.5, 1.5,  0.1,
                0.2, 0.2, 0.2,  0.1                                             //         1.5, 1.5, 1.5,  0.1
            };                                                                  //     };


        // ------------ Control mode selection ------------
        switch (control_mode_)
        {
            case REGULATION:
                regulate_robot(KP, KD, effort_msg);
                break;

            case WHOLE_BODY:
               { 
                    if (!initialized_walking_manager_){
                        initWalkingManagerOnce();
                        start_time_wbc_ = this->now();
                    }

                    const rclcpp::Time t_now = this->now();
                    const double t_mpc = (t_now - start_time_wbc_).seconds();


                    // ------------ Walking manager control ------------
                    // auto start = std::chrono::high_resolution_clock::now();

                    labrob::JointCommand tau;
                    labrob::JointCommand qdd;
                    walking_manager_.update(robot_state_, tau, qdd, t_mpc * 1000);

                    // auto end_time = std::chrono::high_resolution_clock::now();
                    // const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start).count();
                    
                    // RCLCPP_INFO_THROTTLE(
                    // this->get_logger(), *this->get_clock(), 1,  // ms
                    // "t_mpc: %.3f s | Controller period: %ld us", t_mpc, duration
                    // );


                    if (!fillMsgs(tau, qdd, KP, KD, effort_msg) || security_stop_) {             // redundant but ok
                        if (security_stop_)
                            RCLCPP_WARN(this->get_logger(), "Security stop activated! Sending zero commands.");
                        sendZeroCommand();
                        return;
                    }

                    // ------------ log joint torques ------------
                    joint_eff_log_file_ << t_mpc * 1000 << " ";
                    int idx = 0;
                    for (pinocchio::JointIndex j = 2; static_cast<int>(j) < robot_model_.njoints; ++j, ++idx) {
                        const auto& name = robot_model_.names[j];
                        joint_eff_log_file_ << tau[name] << " ";
                    }
                    joint_eff_log_file_ << std::endl;
                    break;
                }

            default:
                sendZeroCommand();
                return;
        }

        // ------------ Publish effort command ------------
        effort_cmd_pub_->publish(effort_msg);
    }

    enum ControlMode {
        NONE,
        REGULATION,
        WHOLE_BODY
    };

    void securityStopService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        security_stop_ = true;
        res->success = true;
        res->message = "Security stop activated.";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    void regulationModeService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (received_joint_state_){
            control_mode_ = REGULATION;
            security_stop_ = false;
            
            res->success = true;
            res->message = "Regulation mode activated.";
            RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        }
        else{
            res->success = false;
            res->message = "Cannot activate regulation mode. Be sure /joint_state is publishing.";
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        }
    }

    void startFilterService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {    
        if (!start_filter_){
            start_filter_ = true;
            res->success = true;
            res->message = "Start filter activated.";
            RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        } else {
            res->success = false;
            res->message = "Filter already started.";
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        }
    }

    void wholeBodyModeService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (start_filter_ && received_joint_state_){
            control_mode_ = WHOLE_BODY;
            security_stop_ = false;

            res->success = true;
            res->message = "Whole-body mode activated.";
            RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        }
        else{
            res->success = false;
            res->message = "Cannot activate whole-body mode. Start the filter first and be sure /joint_state is publishing.";
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        }
    }



    // initialize subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    
    // initialize publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_effort_cmd_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_filtered_state_;
    
    // initialize services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr security_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr regulation_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_filter_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr whole_body_mode_srv_;
    
    // state machine variables
    bool security_stop_ = false;
    bool received_joint_state_ = false;
    bool start_filter_ = false;
    bool initialized_filter = false;
    bool initialized_walking_manager_ = false;
    ControlMode control_mode_;
    
    // time variables
    rclcpp::Time start_time_wbc_;
    rclcpp::Time start_time_filter_;
    rclcpp::Time t_prev_;
    double nominal_dt_ = 0.002;                             // controller nominal period
    
    // controller class
    labrob::WalkingManager walking_manager_;
    
    RobotSensors robot_sensor_;
    labrob::RobotState robot_state_;

    pinocchio::Model robot_model_;
    pinocchio::Data robot_data_;      
    pinocchio::FrameIndex right_leg4_idx_;
    pinocchio::FrameIndex left_leg4_idx_;

    // state estimation classes
    std::shared_ptr<labrob::KF> state_filter_ptr_;
    std::shared_ptr<labrob::RobotOdometry> robot_odometry_ptr_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // log files
    std::ofstream odom_log_;
    std::ofstream imu_log_;
    std::ofstream joint_state_log_;
    std::ofstream csv;
    std::ofstream robot_odom_log;
    std::ofstream joint_eff_log_file_;
    std::ofstream wheel_log_;
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