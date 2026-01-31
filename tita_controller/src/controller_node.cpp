#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <fstream>
#include <iomanip> 
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <StateFilter.hpp>
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
            std::chrono::milliseconds(1),   // filter at 1000 Hz
            std::bind(&RobotController::publish_filtered_state, this));
            
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        // Publisher for /tita_hw/effort_controller/commands
        // effort_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/effort_controller/commands", 10);
        // timer_effort_cmd_ = this->create_wall_timer(
        //     std::chrono::milliseconds(2),   // controller at 500 Hz
        //     std::bind(&RobotController::publish_joint_command, this));

        position_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_position/commands", 10);
        velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_velocity/commands", 10);
        effort_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_effort/commands", 10);
        kp_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_kp/commands", 10);
        kd_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/forward_kd/commands", 10);
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
        right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
        left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
    


        // ------------------ Filter init ----------------------------
        Eigen::Vector<double, 18> x0;
        x0.setZero();
        x0(2) = 0.4;
        x0(7) = 0.28;
        x0(10) = -0.28;
        state_filter_ptr_ = std::make_shared<labrob::KF>(x0, robot_model_);
        

        // Logging
        csv.open("kf_test.csv");
        csv << "t,"
            << "p_odom_x,p_odom_y,p_odom_z,"
            << "p_est_x,p_est_y,p_est_z,"
            << "v_est_x,v_est_y,v_est_z,"
            << "p_cL_est_x,p_cL_est_y,p_cL_est_z,"
            << "p_cR_est_x,p_cR_est_y,p_cR_est_z,"
            << "ba_est_x,ba_est_y,ba_est_z,"
            << "bw_est_x,bw_est_y,bw_est_z\n";
    

        // open log files
        odom_log_.open("odom.txt");
        imu_log_.open("imu_log.txt");
        joint_state_log_.open("joint_state_log.txt");
        // // TODO: log files
        // std::ofstream joint_vel_log_file("/tmp/joint_vel.txt");
        joint_eff_log_file_.open("/tmp/joint_eff.txt");
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

    }

    void publish_filtered_state()
    {
        Eigen::Vector<double, 12> filter_params;
        filter_params.setZero();
        filter_params.segment<4>(0) = robot_sensor_.imu.orientation.coeffs();
        
        Eigen::Vector<double, 14> filter_input;
        filter_input.setZero();
        filter_input.segment<3>(0) = robot_sensor_.imu.linear_acceleration;
        filter_input.segment<3>(3) = robot_sensor_.imu.angular_velocity;

        for(pinocchio::JointIndex joint_id = 2; static_cast<int>(joint_id) < robot_model_.njoints; ++joint_id){
            const std::string& name = robot_model_.names[joint_id];                                         // get joint name from Pinocchio
            filter_params(4 + joint_id - 2) = robot_sensor_.joints[name].pos;                               // Joint positions
            filter_input(6 + joint_id - 2) = robot_sensor_.joints[name].vel;                                // Joint velocities
        }
        
        // Get current ROS time
        rclcpp::Time t_now = this->now();
        if (first_callback_) {
            start_time_filter_ = t_now;             // store initial time
            first_callback_ = false;
            t_prev_ = start_time_filter_;
        }
        double dt = (t_now - t_prev_).seconds();    // compute effective dt
        
        // --------- Run filter ---------
        Eigen::Vector<double,18> filtered_state = state_filter_ptr_->compute_KF_estimate(filter_input, filter_params, dt);
        t_prev_ = t_now;

        // Compute relative time
        double t_rel = (t_now - start_time_filter_).seconds();
        
        // ---- Log ---------------------
        csv << std::fixed << std::setprecision(9);
        csv << t_rel << ","
            << robot_sensor_.odom.position(0) << "," << robot_sensor_.odom.position(1) << "," << robot_sensor_.odom.position(2) << ","
            << filtered_state(0) << "," << filtered_state(1) << "," << filtered_state(2) << ","
            << filtered_state(3) << "," << filtered_state(4) << "," << filtered_state(5) << ","
            << filtered_state(6) << "," << filtered_state(7) << "," << filtered_state(8) << ","
            << filtered_state(9) << "," << filtered_state(10) << "," << filtered_state(11) << ","
            << filtered_state(12) << "," << filtered_state(13) << "," << filtered_state(14) << ","
            << filtered_state(15) << "," << filtered_state(16) << "," << filtered_state(17) << "\n";


        // ------ fill robot state -------
        robot_state_.position = filtered_state.segment<3>(0);
        robot_state_.orientation.coeffs() = filter_params.segment<4>(0);
        robot_state_.linear_velocity = robot_state_.orientation.toRotationMatrix().transpose() * filtered_state.segment<3>(3);
        robot_state_.angular_velocity = filter_input.segment<3>(3);

        for(pinocchio::JointIndex joint_id = 2; static_cast<int>(joint_id) < robot_model_.njoints; ++joint_id){
            const std::string& name = robot_model_.names[joint_id];                                 // get joint name from Pinocchio
            robot_state_.joint_state[name].pos = filter_params(4 + joint_id - 2);
            robot_state_.joint_state[name].vel = filter_input(6 + joint_id - 2);
        }


        // Publish odometry message
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";        // world frame
        msg.child_frame_id = "base";    // robot frame

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
        odom_tf.header.frame_id = "odom";        // world frame
        odom_tf.child_frame_id = "base";    // robot frame
        odom_tf.transform.translation.x = robot_state_.position.x();
        odom_tf.transform.translation.y = robot_state_.position.y();
        odom_tf.transform.translation.z = robot_state_.position.z();

        odom_tf.transform.rotation.x = robot_state_.orientation.x();
        odom_tf.transform.rotation.y = robot_state_.orientation.y();
        odom_tf.transform.rotation.z = robot_state_.orientation.z();
        odom_tf.transform.rotation.w = robot_state_.orientation.w();
        tf_broadcaster_->sendTransform(odom_tf);



        Eigen::Vector<double, 8> q_joint = filter_params.tail<8>();
        Eigen::Quaterniond q_base;
        q_base.coeffs() << filter_params(0), filter_params(1), filter_params(2), filter_params(3);  // (x,y,z,w)
        q_base.normalize();
        Eigen::Vector<double, 3 + 4 + 8> q;
        q << Eigen::Vector3d(0,0,0), 
        q_base.coeffs(),
        q_joint;
        
        // Compute pinocchio terms
        pinocchio::framesForwardKinematics(robot_model_, robot_data_, q); // update robot_data_.oMf
        pinocchio::computeJointJacobians(robot_model_, robot_data_, q);   // compute joint jacobians

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

        p_cL_tf.transform.rotation.x = q_p_cL.x();
        p_cL_tf.transform.rotation.y = q_p_cL.y();
        p_cL_tf.transform.rotation.z = q_p_cL.z();
        p_cL_tf.transform.rotation.w = q_p_cL.w();
        tf_broadcaster_->sendTransform(p_cL_tf);

        geometry_msgs::msg::TransformStamped p_cR_tf;
        p_cR_tf.header.stamp = this->now();
        p_cR_tf.header.frame_id = "odom";
        p_cR_tf.child_frame_id = "p_cR";

        p_cR_tf.transform.translation.x = filtered_state(9);
        p_cR_tf.transform.translation.y = filtered_state(10);
        p_cR_tf.transform.translation.z = filtered_state(11);

        p_cR_tf.transform.rotation.x = q_p_cR.x();
        p_cR_tf.transform.rotation.y = q_p_cR.y();
        p_cR_tf.transform.rotation.z = q_p_cR.z();
        p_cR_tf.transform.rotation.w = q_p_cR.w();
        tf_broadcaster_->sendTransform(p_cR_tf);
    }

    void publishFixedGains()
    {
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray kp_msg, kd_msg;

        kp_msg.data.resize(na, 0.0);            // 0.5
        kd_msg.data.resize(na, 0.0);            // 0.2

        // ---------------- only for regulation -------------------------
        kp_msg.data[3] = kp_msg.data[7] = 0.5;
        kd_msg.data[3] = kd_msg.data[7] = 0.2;
        // --------------------------------------------------------------

        kp_cmd_pub_->publish(kp_msg);           // publish kp only once for tita_bridge control
        kd_cmd_pub_->publish(kd_msg);           // publish kd only once for tita_bridge control
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

    bool fillMsgs(
        const labrob::JointCommand& tau,
        const labrob::JointCommand& qdd,
        std_msgs::msg::Float64MultiArray& position_msg, 
        std_msgs::msg::Float64MultiArray& velocity_msg, 
        std_msgs::msg::Float64MultiArray& effort_msg)
    {
        size_t idx = 0;
        for (pinocchio::JointIndex j = 2; static_cast<int>(j) < robot_model_.njoints; ++j, ++idx) {
            const auto& name = robot_model_.names[j];
            const double u = tau[name];
            const double a = qdd[name];

            if (!std::isfinite(a)) {
                RCLCPP_ERROR(get_logger(), "NaN/Inf command on joint %s", name.c_str());
                return false;
            }

            // ---------------- only for regulation -------------------------
            if (idx == 3 || idx == 7) {
                effort_msg.data[idx] = u;

                const auto& js = robot_state_.joint_state[name];
                velocity_msg.data[idx] = js.vel + a * nominal_dt_;
                position_msg.data[idx] = js.pos + js.vel * nominal_dt_ + 0.5 * a * nominal_dt_ * nominal_dt_;
            }
            // ---------------------------------------------------------------

            // effort_msg.data[idx] = u;

            // const auto& js = robot_state_.joint_state[name];
            // velocity_msg.data[idx] = js.vel + a * nominal_dt_;
            // position_msg.data[idx] = js.pos + js.vel * nominal_dt_ + 0.5 * a * nominal_dt_ * nominal_dt_;
        }
        return true;
    }

    void sendZeroCommand()
    {
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray position_msg, velocity_msg, effort_msg, kp_msg, kd_msg;
        position_msg.data.resize(na, 0.0);
        velocity_msg.data.resize(na, 0.0);
        effort_msg.data.resize(na, 0.0);
        kp_msg.data.resize(na, 0.0);
        kd_msg.data.resize(na, 0.0);


        kp_cmd_pub_->publish(kp_msg); 
        kd_cmd_pub_->publish(kd_msg);
        effort_cmd_pub_->publish(effort_msg);
        velocity_cmd_pub_->publish(velocity_msg);
        position_cmd_pub_->publish(position_msg);
    }

    void publish_joint_command()
    {
        const rclcpp::Time t_now = this->now();
        const double t_from_filter = (t_now - start_time_filter_).seconds();
        if (t_from_filter < time_offset_) return;                               // wait until filter has reached convergence

        const double t_mpc = t_from_filter - time_offset_;


        if (!initialized_walking_manager_){
            initWalkingManagerOnce();
            publishFixedGains();
        }


        // --------- Walking manager control ------------
        auto start = std::chrono::high_resolution_clock::now();

        labrob::JointCommand tau;
        labrob::JointCommand qdd;
        walking_manager_.update(robot_state_, tau, qdd, t_mpc * 1000);

        auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start).count();
 
        RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1,  // ms
        "t_mpc: %.3f s | Controller period: %ld us", t_mpc, duration
        );

        // ---------- Messages fill ----------
        const size_t na = robot_model_.njoints - 2;
        std_msgs::msg::Float64MultiArray position_msg, velocity_msg, effort_msg, kp_msg, kd_msg;
        position_msg.data.resize(na, 0.0);
        velocity_msg.data.resize(na, 0.0);
        effort_msg.data.resize(na, 0.0);
        kp_msg.data.resize(na, 0.0);
        kd_msg.data.resize(na, 0.0);

        if (!fillMsgs(tau, qdd, position_msg, velocity_msg, effort_msg)) {
            sendZeroCommand();
            return;
        }
      
        // ---------- Publish ----------
        position_cmd_pub_->publish(position_msg);
        velocity_cmd_pub_->publish(velocity_msg);
        effort_cmd_pub_->publish(effort_msg);
    }




    // initialize subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    // initialize publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kp_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kd_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_effort_cmd_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_filtered_state_;

    
    RobotSensors robot_sensor_;
    labrob::RobotState robot_state_;

    bool initialized_walking_manager_ = false;
    labrob::WalkingManager walking_manager_;

    bool first_callback_ = true;
    rclcpp::Time start_time_filter_;
    rclcpp::Time t_prev_;
    double time_offset_ = 5;                    // start the controller with an offset after the filter start
    double nominal_dt_ = 0.002;                 // controller nomianal period

    pinocchio::Model robot_model_;
    pinocchio::Data robot_data_;      
    pinocchio::FrameIndex right_leg4_idx_;
    pinocchio::FrameIndex left_leg4_idx_;

    std::shared_ptr<labrob::KF> state_filter_ptr_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // log files
    std::ofstream odom_log_;
    std::ofstream imu_log_;
    std::ofstream joint_state_log_;
    std::ofstream csv;
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
