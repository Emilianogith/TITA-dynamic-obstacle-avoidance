#include <WalkingManager.hpp>


namespace labrob {

bool WalkingManager::init(const labrob::RobotState& initial_robot_state,
                     std::map<std::string, double> &armatures) {
    
    // Read URDF from file:
    std::string robot_description_filename = "../tita_description/tita.urdf";

    // Build Pinocchio model and data from URDF:
    pinocchio::Model full_robot_model;
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(
        robot_description_filename,
        root_joint,
        full_robot_model
    );
    const std::vector<std::string> joint_to_lock_names{};
    std::vector<pinocchio::JointIndex> joint_ids_to_lock;
    for (const auto& joint_name : joint_to_lock_names) {
        if (full_robot_model.existJointName(joint_name)) {
        joint_ids_to_lock.push_back(full_robot_model.getJointId(joint_name));
        }
    }

    robot_model_ = pinocchio::buildReducedModel(
        full_robot_model,
        joint_ids_to_lock,
        pinocchio::neutral(full_robot_model)
    );
    robot_data_ = pinocchio::Data(robot_model_);
    
    right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
    left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");

    int njnt = robot_model_.nv - 6;

    // TODO: init using node handle.
    controller_frequency_ = 500;                        // CONTROLLA!!!!!!
    controller_timestep_msec_ = 1000 / controller_frequency_;
    

    // Desired configuration:
    des_configuration_.qjnt = Eigen::VectorXd::Zero(njnt);
    des_configuration_.qjnt << 
    0.0,   // joint_left_leg_1
    0.5,   // joint_left_leg_2
    -1.0,   // joint_left_leg_3
    0.0,   // joint_left_leg_4
    0.0,   // joint_right_leg_1
    0.5,   // joint_right_leg_2
    -1.0,   // joint_right_leg_3
    0.0;   // joint_right_leg_4
    des_configuration_.qjntdot = Eigen::VectorXd::Zero(njnt);
    des_configuration_.qjntddot = Eigen::VectorXd::Zero(njnt);
    // des_configuration_.position = Eigen::Vector3d(0.0, 0.0, 0.2);
    // des_configuration_.orientation = Eigen::Quaterniond::Identity();
    // des_configuration_.linear_velocity = Eigen::Vector3d::Zero();
    // des_configuration_.angular_velocity = Eigen::Vector3d::Zero();
    des_configuration_.com.pos = Eigen::Vector3d(0.0, 0.0, 0.40);  
    des_configuration_.com.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    des_configuration_.com.acc = Eigen::Vector3d::Zero();
    des_configuration_.lwheel_contact.pos.p = Eigen::Vector3d(0.0, 0.283, 0.0925);
    des_configuration_.lwheel_contact.pos.R = Eigen::Matrix3d::Identity();     // desired orientation of the contact frame
    // des_configuration_.lwheel_contact.pos.R << 1,0,0,  0, 0.984,0.1736,  -0.1736,0.984,0;
    des_configuration_.lwheel_contact.vel = Eigen::Vector<double, 6>::Zero();
    des_configuration_.lwheel_contact.acc = Eigen::Vector<double, 6>::Zero();
    des_configuration_.rwheel_contact.pos.p = Eigen::Vector3d(0.0, -0.283, 0.0925);
    des_configuration_.rwheel_contact.pos.R = Eigen::Matrix3d::Identity();
    // des_configuration_.rwheel_contact.pos.R << 1,0,0,  0, 0.984,0.1736,  -0.1736,0.984,0;
    des_configuration_.rwheel_contact.vel = Eigen::Vector<double, 6>::Zero();
    des_configuration_.rwheel_contact.acc = Eigen::Vector<double, 6>::Zero();
    des_configuration_.base_link.pos =Eigen::Matrix3d::Identity();
    // des_configuration_.base_link.pos << 0,-1,0,  1,0,0,  0,0,1;
    des_configuration_.base_link.vel = Eigen::Vector3d::Zero();
    des_configuration_.base_link.acc = Eigen::Vector3d::Zero();
    des_configuration_.in_contact = false;


  

    // Init MPC:
    // Eigen::Vector3d p_CoM = robot_data_.com[0];
    // int64_t mpc_prediction_horizon_msec = 2000;
    // int64_t mpc_timestep_msec = 100;
    // double com_target_height = p_CoM.z() - T_lsole_init.translation().z();
    // double foot_constraint_square_length = 100; //0.20;
    // double foot_constraint_square_width = 100; //0.07;
    // Eigen::Vector3d p_ZMP = p_CoM - Eigen::Vector3d(0.0, 0.0, com_target_height);
    // filtered_state_ = labrob::LIPState(
    //     p_CoM,
    //     Eigen::Vector3d::Zero(),
    //     p_ZMP
    // );

    mpc_.set_pendulum_height(des_configuration_.com.pos(2));
    zmp_ref = Eigen::Matrix<double, 1, 400+1>::Zero();




    auto params = WholeBodyControllerParams::getDefaultParams();
    // params.Kp_motion = 13000;                   
    // params.Kd_motion = 300;                    
    // params.Kp_regulation = 1000.0;            
    // params.Kd_regulation = 50;                 

    // params.Kp_wheel = 90000.0;                 
    // params.Kd_wheel = 200.0;                 

    // params.weight_q_ddot = 1e-4;                
    // params.weight_com = 1.1;                    
    // params.weight_lwheel = 2.0;                 
    // params.weight_rwheel = 2.0;                 
    // params.weight_base = 0.1;                  
    // params.weight_angular_momentum = 0.0001;    
    // params.weight_regulation = 0.0; 

    // params.cmm_selection_matrix_x = 1e-6;       
    // params.cmm_selection_matrix_y = 1e-6;       
    // params.cmm_selection_matrix_z = 1e-4;

    // params.gamma = 10;                          
    // params.mu = 0.5;                            

    whole_body_controller_ptr_ = std::make_shared<labrob::WholeBodyController>(
        params,
        robot_model_,
        initial_robot_state,
        0.001 * controller_timestep_msec_,
        armatures
    );

    // Init log files:
    // TODO: may be better to use a proper logging system such as glog.
    state_log_file_.open("/tmp/state_log_file.txt");
    state_log_file_ << "time,"
         << "com_x,com_y,com_z,"
         << "com_x_des,com_y_des,com_z_des,"
         << "wheel_l_x,wheel_l_y,wheel_l_z,"
         << "wheel_l_x_des,wheel_l_y_des,wheel_l_z_des,"
         << "wheel_r_x,wheel_r_y,wheel_r_z,"
         << "wheel_r_x_des,wheel_r_y_des,wheel_r_z_des"
         << std::endl;

    return true;
    }



void WalkingManager::update(
    const labrob::RobotState& robot_state,
    labrob::JointCommand& joint_command) {

    auto start_time = std::chrono::system_clock::now();

    auto q = robot_state_to_pinocchio_joint_configuration(robot_model_, robot_state);
    auto qdot = robot_state_to_pinocchio_joint_velocity(robot_model_, robot_state);

    // // Perform forward kinematics on the whole tree and update robot data:
    // pinocchio::forwardKinematics(robot_model_, robot_data_, q);

    // // NOTE: jacobianCenterOfMass calls forwardKinematics and
    // //       computeJointJacobians.
    // pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q);
    // pinocchio::computeJointJacobiansTimeVariation(robot_model_, robot_data_, q, qdot);
    pinocchio::framesForwardKinematics(robot_model_, robot_data_, q);
    pinocchio::centerOfMass(robot_model_, robot_data_, q, qdot, 0.0 * qdot); // This is used to compute the CoM drift (J_com_dot * qdot)
    const auto& centroidal_momentum_matrix = pinocchio::ccrba(
        robot_model_,
        robot_data_,
        q,
        qdot
    );
    // auto angular_momentum = (centroidal_momentum_matrix * qdot).tail<3>();

    const auto& p_CoM = robot_data_.com[0];
    const auto& v_CoM = robot_data_.vcom[0];
    const auto& r_wheel_center = robot_data_.oMf[right_leg4_idx_];
    const auto& l_wheel_center = robot_data_.oMf[left_leg4_idx_];
    Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center.translation(), r_wheel_center.rotation(), 0.0925);
    Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center.translation(), l_wheel_center.rotation(), 0.0925);
    Eigen::Vector3d right_contact = r_wheel_center.translation() + right_rCP;
    Eigen::Vector3d left_contact = l_wheel_center.translation() + left_rCP;

    // const auto& a_CoM_drift = robot_data_.acom[0];
    // const auto& J_CoM = robot_data_.Jcom;
    // const auto& T_torso = robot_data_.oMf[torso_idx_];
    // const auto& T_pelvis = robot_data_.oMf[pelvis_idx_];
    // auto torso_orientation = T_torso.rotation();
    // auto pelvis_orientation = T_pelvis.rotation();
    // Eigen::MatrixXd J_torso = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // pinocchio::getFrameJacobian(
    //     robot_model_,
    //     robot_data_,
    //     torso_idx_,
    //     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    //     J_torso
    // );



    // // LQR-based MPC
    // auto start_time_LQR = std::chrono::high_resolution_clock::now();
    // labrob::LQR lqr(des_configuration_.com.pos(2));
    // const double& x_com = p_CoM(0);
    // const double& vx_com = v_CoM(0);
    // const double& ax_com = robot_data_.acom[0](0);
    // const double& x_prev_zmp = des_configuration_.lwheel_contact.pos.p(0);
    // const double& vx_prev_zmp = des_configuration_.lwheel_contact.vel(0);
    // const double& ax_prev_zmp = des_configuration_.lwheel_contact.acc(0);

    // lqr.solve(x_com, vx_com, ax_com, x_prev_zmp, vx_prev_zmp, ax_prev_zmp);
    // SolutionLQR sol = lqr.get_solution();

    // auto end_time_LQR = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_time_LQR = (end_time_LQR - start_time_LQR) * 1000;
    // std::cout << "LQR solve took: " << elapsed_time_LQR.count() << " ms" << std::endl;

    // if (std::fabs(t_msec_ - 9360.0) < 0.5){
    //     lqr.record_logs(t_msec_);
    // }
   
    // des_configuration_.com.pos(0) = sol.com.pos;  
    // des_configuration_.com.vel(0) = sol.com.vel;
    // des_configuration_.com.acc(0) = sol.com.acc; 

    // des_configuration_.lwheel_contact.pos.p(0) = sol.zmp.pos;
    // des_configuration_.lwheel_contact.pos.p(1) = left_contact(1);
    // des_configuration_.lwheel_contact.vel(0) = sol.zmp.vel;
    // des_configuration_.lwheel_contact.acc(0) = sol.zmp.acc;

    // des_configuration_.rwheel_contact.pos.p(0) = sol.zmp.pos;
    // des_configuration_.rwheel_contact.pos.p(1) = right_contact(1);
    // des_configuration_.rwheel_contact.vel(0) = sol.zmp.vel;
    // des_configuration_.rwheel_contact.acc(0) = sol.zmp.acc;


    
    
    
    // squatting manouver
    // if (std::fabs(t_msec_) < 1000.0){
    //     des_configuration_.com.pos(2) -= 0.0001;
    // }else if (std::fabs(t_msec_) > 4000.0 && std::fabs(t_msec_) < 5000.0){
    //     des_configuration_.com.pos(2) += 0.0001;
    // }
    // mpc_.set_pendulum_height(des_configuration_.com.pos(2));



    // compute reference trajectory
    // double x_zmp_curr = l_wheel_center.translation()(0);
    // double step_x = 0.2; 
    // for (int i = 0; i < 400+1; ++i)
    //     {
    //         int step_index = floor(i / 100);
    //         zmp_ref(0,i) = x_zmp_curr + step_x * step_index;
    //     }

    // mpc_.set_reference_trajectory(zmp_ref);
    
    
    // DDP-based MPC 
    if (std::fabs(t_msec_ - 4312.0) < 0.5){
            mpc_.record_logs = true;
        }          
    auto start_time_mpc = std::chrono::high_resolution_clock::now();
    // Eigen::Vector<double, 9> x0 = Eigen::Vector<double, 9>::Zero();
    // // x0 << 0.0, 0.0, 0.0, 0.11, 0.0, 0.1, 0.075, 0.0, 0.0;
    // x0 << p_CoM(0), v_CoM(0), l_wheel_center.translation()(0),
    // p_CoM(1), v_CoM(1), l_wheel_center.translation()(1),
    // p_CoM(2), v_CoM(2), l_wheel_center.translation()(2);

    Eigen::Vector<double, 3> x0 = Eigen::Vector<double, 3>::Zero();
    x0 << p_CoM(0), v_CoM(0), l_wheel_center.translation()(0);

    std::cout << "x0" << x0 << std::endl;
    
    Eigen::MatrixXd J_right_wheel = Eigen::MatrixXd::Zero(6, robot_model_.nv);;
    pinocchio::getFrameJacobian(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel);
    Eigen::Vector<double, 6> current_rwheel_contact_vel = J_right_wheel * qdot;
    Eigen::Vector3d curr_zmp_vel = current_rwheel_contact_vel.head<3>();
    

    mpc_.solve(x0, curr_zmp_vel);

    SolutionMPC sol = mpc_.get_solution();

    auto end_time_mpc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time_mpc = (end_time_mpc - start_time_mpc) * 1000;
    std::cout << "MPC solve took: " << elapsed_time_mpc.count() << " ms" << std::endl;

    des_configuration_.com.pos(0) = sol.com.pos(0);  
    des_configuration_.com.vel(0) = sol.com.vel(0);
    des_configuration_.com.acc(0) = sol.com.acc(0); 

    des_configuration_.lwheel_contact.pos.p(0) = sol.zmp.pos(0);
    des_configuration_.lwheel_contact.pos.p(1) = left_contact(1);
    des_configuration_.lwheel_contact.vel(0) = sol.zmp.vel(0);
    des_configuration_.lwheel_contact.acc(0) = sol.zmp.acc(0);

    des_configuration_.rwheel_contact.pos.p(0) = sol.zmp.pos(0);
    des_configuration_.rwheel_contact.pos.p(1) = right_contact(1);
    des_configuration_.rwheel_contact.vel(0) = sol.zmp.vel(0);
    des_configuration_.rwheel_contact.acc(0) = sol.zmp.acc(0);


    

    joint_command = whole_body_controller_ptr_->compute_inverse_dynamics(robot_state, des_configuration_);




    auto end_time = std::chrono::system_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << "WalkingManager::update() took " << elapsed_time << " us" << std::endl;
    
    // Update timing in milliseconds.
    // NOTE: assuming update() is actually called every controller_timestep_msec_
    //       milliseconds.
    t_msec_ += controller_timestep_msec_;

    std::cout << "t_msec_ " << t_msec_ << std::endl;

    // Log:
    state_log_file_
        << t_msec_ << ","
        << p_CoM(0) << "," << p_CoM(1) << "," << p_CoM(2) << ","
        << des_configuration_.com.pos(0) << "," << des_configuration_.com.pos(1) << "," << des_configuration_.com.pos(2) << ","
        << l_wheel_center.translation()(0) << "," << l_wheel_center.translation()(1) << "," << l_wheel_center.translation()(2) << ","
        << des_configuration_.lwheel_contact.pos.p(0) << "," << des_configuration_.lwheel_contact.pos.p(1) << "," << des_configuration_.lwheel_contact.pos.p(2) << ","
        << r_wheel_center.translation()(0) << "," << r_wheel_center.translation()(1) << "," << r_wheel_center.translation()(2) << ","
        << des_configuration_.rwheel_contact.pos.p(0) << "," << des_configuration_.rwheel_contact.pos.p(1) << "," << des_configuration_.rwheel_contact.pos.p(2)
        << std::endl;
}

} // end namespace labrob
