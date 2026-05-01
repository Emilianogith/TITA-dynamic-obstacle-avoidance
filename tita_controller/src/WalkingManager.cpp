#include <WalkingManager.hpp>


namespace labrob {

bool WalkingManager::init(const labrob::RobotState& initial_robot_state,
                    std::map<std::string, double> &armatures,
                    const pinocchio::Model& robot_model) {
    
    robot_model_ = &robot_model;
    robot_data_ = pinocchio::Data(*robot_model_);
    
    base_idx_ = robot_model_->getFrameId("base_link");
    right_leg4_idx_ = robot_model_->getFrameId("right_leg_4");
    left_leg4_idx_ = robot_model_->getFrameId("left_leg_4");

    int njnt = robot_model_->nv - 6;

    // TODO: init using node handle.
    controller_frequency_ = 500;                                    // nominal control frequency 
    controller_timestep_msec_ = 1000 / controller_frequency_;
    


    auto q = robot_state_to_pinocchio_joint_configuration(*robot_model_, initial_robot_state);
    auto qdot = robot_state_to_pinocchio_joint_velocity(*robot_model_, initial_robot_state);

    pinocchio::centerOfMass(*robot_model_, robot_data_, q, qdot);      // compute com pos and vel
    pinocchio::framesForwardKinematics(*robot_model_, robot_data_, q); // update robot_data_.oMf
    pinocchio::computeJointJacobians(*robot_model_, robot_data_, q);   // compute joint jacobians


    const auto& base_frame = robot_data_.oMf[base_idx_]; 
    Eigen::Matrix3d R_base = base_frame.rotation();

    wheel_radius_ = 0.0925;
    // Desired configuration:
    des_configuration_.tau_prev = Eigen::VectorXd::Zero(njnt);
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
    des_configuration_.com.pos = Eigen::Vector3d(0.0, 0.0, 0.4);  
    des_configuration_.com.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    des_configuration_.com.acc = Eigen::Vector3d::Zero();
    des_configuration_.lwheel.pos.p = Eigen::Vector3d(0.0, 0.2835, wheel_radius_);
    des_configuration_.lwheel.pos.R = Eigen::Matrix3d::Identity();     
    des_configuration_.lwheel.vel = Eigen::Vector<double, 6>::Zero();
    des_configuration_.lwheel.acc = Eigen::Vector<double, 6>::Zero();
    des_configuration_.rwheel.pos.p = Eigen::Vector3d(0.0, -0.2835, wheel_radius_);
    des_configuration_.rwheel.pos.R = Eigen::Matrix3d::Identity();
    des_configuration_.rwheel.vel = Eigen::Vector<double, 6>::Zero();
    des_configuration_.rwheel.acc = Eigen::Vector<double, 6>::Zero();
    des_configuration_.base_link.pos = R_base;
    des_configuration_.base_link.vel = Eigen::Vector3d::Zero();
    des_configuration_.base_link.acc = Eigen::Vector3d::Zero();
    des_configuration_.in_contact = true;


    // Init WBC:
    auto params = WholeBodyControllerParams::getDefaultParams();
    whole_body_controller_ptr_ = std::make_shared<labrob::WholeBodyController>(
        params,
        *robot_model_,
        0.001 * controller_timestep_msec_,
        armatures
    );

    


    // Init MPC:
    Eigen::Vector3d p_CoM = robot_data_.com[0];
    Eigen::Vector3d v_CoM = robot_data_.vcom[0];
    const auto& r_wheel_center = robot_data_.oMf[right_leg4_idx_];
    const auto& l_wheel_center = robot_data_.oMf[left_leg4_idx_];
    Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center.rotation(), whole_body_controller_ptr_->wheel_radius_);
    Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center.rotation(), whole_body_controller_ptr_->wheel_radius_);
    Eigen::Vector3d right_contact = r_wheel_center.translation() + right_rCP;
    Eigen::Vector3d left_contact = l_wheel_center.translation() + left_rCP;

    Eigen::MatrixXd J_left_wheel = Eigen::MatrixXd::Zero(6, robot_model_->nv);;
    pinocchio::getFrameJacobian(*robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel);
    Eigen::Vector<double, 6> current_lwheel_vel = J_left_wheel * qdot;
    Eigen::Vector3d curr_pl_vel = current_lwheel_vel.head<3>();

    Eigen::MatrixXd J_right_wheel = Eigen::MatrixXd::Zero(6, robot_model_->nv);;
    pinocchio::getFrameJacobian(*robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel);
    Eigen::Vector<double, 6> current_rwheel_vel = J_right_wheel * qdot;
    Eigen::Vector3d curr_pr_vel = current_rwheel_vel.head<3>();


    Eigen::Vector3d diff = left_contact - right_contact;
    double theta = atan2(-diff.x(), diff.y());


    // plan the offline trajectory
    walkingPlanner_.offline_plan(0.001 * controller_timestep_msec_, false, p_CoM, theta);


    // init LQR
    // double z_des = p_CoM(2) + 0.05;
    // des_configuration_.com.pos(2) = std::clamp(z_des, 0.25, 0.4);
    // lqr_.init(des_configuration_.com.pos(2));

    // initialize the MPC
    Eigen::VectorXd x_IN(18);
    x_IN.segment<3>(0) = p_CoM;
    x_IN.segment<3>(3) = v_CoM;
    x_IN.segment<3>(6) = left_contact;
    x_IN.segment<3>(9) = right_contact;
    x_IN.segment<3>(12) = curr_pl_vel;
    x_IN.segment<3>(15) = curr_pr_vel;
    mpc_.set_planner(walkingPlanner_, 0.001 * controller_timestep_msec_);
    mpc_.init_solver(x_IN);
    // mpc_.solve(x_IN);

    // Init logger
    logger_.reserve(20000);

    return true;
    }



void WalkingManager::update(
    const labrob::RobotState& robot_state,
    labrob::JointCommand& joint_torque, 
    labrob::JointCommand& joint_acceleration,
    double t_msec_) {

  
    auto start_time = std::chrono::system_clock::now();

    auto q = robot_state_to_pinocchio_joint_configuration(*robot_model_, robot_state);
    auto qdot = robot_state_to_pinocchio_joint_velocity(*robot_model_, robot_state);

    pinocchio::centerOfMass(*robot_model_, robot_data_, q, qdot);      // compute com pos and vel
    pinocchio::framesForwardKinematics(*robot_model_, robot_data_, q); // update robot_data_.oMf
    pinocchio::computeJointJacobians(*robot_model_, robot_data_, q);   // compute joint jacobians

    const auto& p_CoM = robot_data_.com[0];
    const auto& v_CoM = robot_data_.vcom[0];
    const auto& r_wheel_center = robot_data_.oMf[right_leg4_idx_];
    const auto& l_wheel_center = robot_data_.oMf[left_leg4_idx_];
    Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center.rotation(), whole_body_controller_ptr_->wheel_radius_);
    Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center.rotation(), whole_body_controller_ptr_->wheel_radius_);
    Eigen::Vector3d right_contact = r_wheel_center.translation() + right_rCP;
    Eigen::Vector3d left_contact = l_wheel_center.translation() + left_rCP;

    Eigen::MatrixXd J_left_wheel = Eigen::MatrixXd::Zero(6, robot_model_->nv);;
    pinocchio::getFrameJacobian(*robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel);
    Eigen::Vector<double, 6> current_lwheel_vel = J_left_wheel * qdot;
    Eigen::Vector3d curr_pl_vel = current_lwheel_vel.head<3>();

    Eigen::MatrixXd J_right_wheel = Eigen::MatrixXd::Zero(6, robot_model_->nv);;
    pinocchio::getFrameJacobian(*robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel);
    Eigen::Vector<double, 6> current_rwheel_vel = J_right_wheel * qdot;
    Eigen::Vector3d curr_pr_vel = current_rwheel_vel.head<3>();



    // // LQR-based MPC
    // auto start_time_LQR = std::chrono::high_resolution_clock::now();

    // const auto& base_frame = robot_data_.oMf[base_idx_]; 
    // Eigen::Matrix3d R_base = base_frame.rotation();

    // Eigen::Vector3d p_CoM_base = R_base.transpose() * p_CoM;
    // Eigen::Vector3d v_CoM_base = R_base.transpose() * v_CoM;

    // Eigen::Vector3d p_lwheel_base = R_base.transpose() * l_wheel_center.translation();
    // Eigen::Vector3d v_lwheel_base = R_base.transpose() * curr_pl_vel.segment<3>(0);

    // lqr_.solve(p_CoM_base(0), v_CoM_base(0), p_lwheel_base(0), v_lwheel_base(0));
    // SolutionLQR sol_lqr = lqr_.get_solution();

    // auto end_time_LQR = std::chrono::high_resolution_clock::now();
    // auto elapsed_time_LQR = std::chrono::duration<double, std::milli>(end_time_LQR - start_time_LQR).count();
    // // std::cout << "LQR solve took: " << elapsed_time_LQR << " ms" << std::endl;

    // // if (cycle_counter == 500){
    // //     lqr_.record_logs(t_msec_);
    // // }
   
    // Eigen::Vector3d sol_com_pos = Eigen::Vector3d(sol_lqr.com.pos, 0, des_configuration_.com.pos(2));
    // Eigen::Vector3d sol_com_vel = Eigen::Vector3d(sol_lqr.com.vel, 0, 0);
    // Eigen::Vector3d sol_com_acc = Eigen::Vector3d(sol_lqr.com.acc, 0, 0);


    // Eigen::Vector3d sol_pl_pos = Eigen::Vector3d(sol_lqr.com.pos, 0.2835, 0);
    // Eigen::Vector3d sol_pl_vel = Eigen::Vector3d(sol_lqr.com.vel, 0, 0);
    // Eigen::Vector3d sol_pl_acc = Eigen::Vector3d(sol_lqr.com.acc, 0, 0);

    // Eigen::Vector3d sol_pr_pos = Eigen::Vector3d(sol_lqr.com.pos, -0.2835, 0);
    // Eigen::Vector3d sol_pr_vel = Eigen::Vector3d(sol_lqr.com.vel, 0, 0);
    // Eigen::Vector3d sol_pr_acc = Eigen::Vector3d(sol_lqr.com.acc, 0, 0);

    // des_configuration_.com.pos.segment<2>(0) = (R_base * sol_com_pos).segment<2>(0);  
    // des_configuration_.com.vel.segment<2>(0) = (R_base * sol_com_vel).segment<2>(0); 
    // des_configuration_.com.acc.segment<2>(0) = (R_base * sol_com_acc).segment<2>(0); 

    // des_configuration_.lwheel.pos.p.segment<2>(0) = (R_base * sol_pl_pos).segment<2>(0);  
    // des_configuration_.lwheel.pos.p(2) = wheel_radius_;
    // des_configuration_.lwheel.vel.segment<2>(0) = (R_base * sol_pl_vel).segment<2>(0);
    // des_configuration_.lwheel.acc.segment<2>(0) = (R_base * sol_pl_acc).segment<2>(0);

    // des_configuration_.rwheel.pos.p.segment<2>(0) = (R_base * sol_pr_pos).segment<2>(0);  
    // des_configuration_.rwheel.pos.p(2) = wheel_radius_;
    // des_configuration_.rwheel.vel.segment<2>(0) = (R_base * sol_pr_vel).segment<2>(0);
    // des_configuration_.rwheel.acc.segment<2>(0) = (R_base * sol_pr_acc).segment<2>(0);





    // ---------------- jump routine -------------------------

    // maximum height obstacle
    // if (std::fabs(t_msec_ - 2000.0) < 0.5){
    //     walkingPlanner_.jumpRoutine(t_msec_, 0.31, 0.4);    // displacement com = 0.31; displacement leg = 0.4; 
    // }

    // high speed obstacle
    // if (std::fabs(t_msec_ - 4000.0) < 0.5){
    //     walkingPlanner_.jumpRoutine(t_msec_, 0.21, 0.3);
    // }


    // 3-obstacle
    // if (std::fabs(t_msec_ - 1230.0) < 0.5){
    //     walkingPlanner_.jumpRoutine(t_msec_, 0.15);
    // }

    // if (std::fabs(t_msec_ - 2320.0) < 0.5){
    //     walkingPlanner_.jumpRoutine(t_msec_, 0.25);
    // }

    // if (std::fabs(t_msec_ - 3800.0) < 0.5){
    //     walkingPlanner_.jumpRoutine(t_msec_, 0.30);
    // }
    // --------------------------------------------------------

    
    // DFIP (DDP) - based MPC
    Eigen::VectorXd x_IN(18);
    x_IN.segment<3>(0) = p_CoM;
    x_IN.segment<3>(3) = v_CoM;
    x_IN.segment<3>(6) = left_contact;
    x_IN.segment<3>(9) = right_contact;
    x_IN.segment<3>(12) = curr_pl_vel;
    x_IN.segment<3>(15) = curr_pr_vel;
    
    mpc_.t_msec = t_msec_;

    auto start_time_mpc = std::chrono::system_clock::now();
    mpc_.solve(x_IN);
    auto end_time_mpc = std::chrono::system_clock::now();
    auto time_mpc = std::chrono::duration_cast<std::chrono::microseconds>(end_time_mpc - start_time_mpc).count();
    // std::cout << "MPC took " << time_mpc << " us" << std::endl;

    SolutionMPC sol = mpc_.get_solution();
    
    des_configuration_.com.pos = sol.com.pos;
    des_configuration_.com.vel = sol.com.vel;
    des_configuration_.com.acc = sol.com.acc;

    // des_configuration_.lwheel.pos.p = l_wheel_center.translation() + sol.pl.vel * mpc_.get_nominal_dt();
    // des_configuration_.lwheel.vel.segment<3>(0) = sol.pl.vel.segment<3>(0);
    // des_configuration_.lwheel.acc.segment<3>(0) = sol.pl.acc.segment<3>(0);

    // des_configuration_.rwheel.pos.p = r_wheel_center.translation() + sol.pr.vel * mpc_.get_nominal_dt();
    // des_configuration_.rwheel.vel.segment<3>(0) = sol.pr.vel.segment<3>(0);
    // des_configuration_.rwheel.acc.segment<3>(0) = sol.pr.acc.segment<3>(0);


    des_configuration_.lwheel.pos.p.segment<2>(0) = sol.pl.pos.segment<2>(0);
    des_configuration_.lwheel.pos.p(2) = sol.pl.pos(2) + wheel_radius_;
    des_configuration_.lwheel.vel.segment<3>(0) = sol.pl.vel.segment<3>(0);
    des_configuration_.lwheel.acc.segment<3>(0) = sol.pl.acc.segment<3>(0);

    des_configuration_.rwheel.pos.p.segment<2>(0) = sol.pr.pos.segment<2>(0);
    des_configuration_.rwheel.pos.p(2) = sol.pr.pos(2) + wheel_radius_;
    des_configuration_.rwheel.vel.segment<3>(0) = sol.pr.vel.segment<3>(0);
    des_configuration_.rwheel.acc.segment<3>(0) = sol.pr.acc.segment<3>(0);


    Eigen::Matrix3d R_theta = Eigen::Matrix3d::Zero();
    R_theta << cos(sol.theta), -sin(sol.theta), 0,
            sin(sol.theta), cos(sol.theta), 0,
            0,0,1;
    des_configuration_.base_link.pos = R_theta;
    des_configuration_.base_link.vel = Eigen::Vector3d(0,0,sol.omega);
    des_configuration_.base_link.acc = Eigen::Vector3d(0,0,sol.alpha);

    // get jumping phase from planned trajectory
        // jump_state = 0 : contact phase
        // jump_state = 1 : pre-jump phase
        // jump_state = 2 : ground-detachment phase
        // jump_state = 3 : flight phase
        // jump_state = 4 : post-jump phase
    // int jump_state = walkingPlanner_.get_jump_phase_at_time_ms(t_msec_);

    // des_configuration_.in_contact = (jump_state == 3) ? false : true;

    // // change WBC params in jump state
    // switch (jump_state) {

    //     case 3: {               // flight phase
    //         auto jump_params = WholeBodyControllerParams::getJumpParams();   
    //         whole_body_controller_ptr_->params_ = jump_params;
    //         break;
    //     }
        
    //     case 1: {               // pre-jump phase                                        
    //         auto params = WholeBodyControllerParams::getDefaultParams();  
    //         whole_body_controller_ptr_->params_ = params;       
    //         break;
    //     }  

    //     case 2: {               // ground-detachment phase
    //         auto params = WholeBodyControllerParams::getDefaultParams(); 

    //         // penalize angular momenutum 
    //         params.weight_angular_momentum = 9.0;
    //         params.cmm_selection_matrix_x = 1000;       
    //         params.cmm_selection_matrix_y = 1000;       
    //         params.cmm_selection_matrix_z = 1e-4;
    //         whole_body_controller_ptr_->params_ = params;  
    //         break; 
    //     }  

    //     case 4: {               // post-jump phase
    //         auto params = WholeBodyControllerParams::getRobustParams();   
    //         whole_body_controller_ptr_->params_ = params;
    //         break;
    //     }

    // }


    auto start_time_wbc = std::chrono::system_clock::now();
    whole_body_controller_ptr_->compute_inverse_dynamics(robot_state, des_configuration_, joint_torque, joint_acceleration);

    auto end_time_wbc = std::chrono::system_clock::now();
    auto time_wbc = std::chrono::duration_cast<std::chrono::microseconds>(end_time_wbc - start_time_wbc).count();
    // std::cout << "WBC took " << time_wbc << " us" << std::endl;

    // for reducing chattering
    int idx = 0;
    for(pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex) robot_model_->njoints; ++joint_id, ++idx) {
        const auto& joint_name = robot_model_->names[joint_id];
        des_configuration_.tau_prev(idx) = joint_torque[joint_name];
    }
    

    auto end_time = std::chrono::system_clock::now();
    auto controller_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    // std::cout << "WalkingManager::update() took " << controller_time << " us" << std::endl;
    
    // std::cout << "t_msec_ " << t_msec_ << std::endl;


    // log WBC data
    labrob::WBCEntry wbc_entry;
    wbc_entry.time_ms = t_msec_;

    wbc_entry.com_x = p_CoM(0); 
    wbc_entry.com_y = p_CoM(1); 
    wbc_entry.com_z = p_CoM(2);

    wbc_entry.com_x_des = des_configuration_.com.pos(0); 
    wbc_entry.com_y_des = des_configuration_.com.pos(1); 
    wbc_entry.com_z_des = des_configuration_.com.pos(2);

    wbc_entry.wheel_l_x = l_wheel_center.translation()(0);
    wbc_entry.wheel_l_y = l_wheel_center.translation()(1);
    wbc_entry.wheel_l_z = l_wheel_center.translation()(2);

    wbc_entry.wheel_l_x_des = des_configuration_.lwheel.pos.p(0);
    wbc_entry.wheel_l_y_des = des_configuration_.lwheel.pos.p(1);
    wbc_entry.wheel_l_z_des = des_configuration_.lwheel.pos.p(2);

    wbc_entry.wheel_r_x = r_wheel_center.translation()(0);
    wbc_entry.wheel_r_y = r_wheel_center.translation()(1);
    wbc_entry.wheel_r_z = r_wheel_center.translation()(2);

    wbc_entry.wheel_r_x_des = des_configuration_.rwheel.pos.p(0);
    wbc_entry.wheel_r_y_des = des_configuration_.rwheel.pos.p(1);
    wbc_entry.wheel_r_z_des = des_configuration_.rwheel.pos.p(2);

    logger_.log_wbc_data(std::move(wbc_entry));

    
    // log MPC data
    if (cycle_counter % 10 == 0){       // save mpc predictions every 10 cycles
        labrob::MPCEntry mpc_entry;
        mpc_entry.time_ms = t_msec_;

        mpc_entry.X = mpc_.X;
        mpc_entry.U = mpc_.U;
    
        logger_.log_mpc_data(std::move(mpc_entry));
    }

    // log timings data
    labrob::TimingEntry timing;

    timing.time_mpc_us = time_mpc;
    // timing.time_mpc_us = elapsed_time_LQR;
    timing.time_wbc_us = time_wbc;
    timing.total_time_us = controller_time;

    logger_.log_timing_data(std::move(timing));

    cycle_counter++;

}

void WalkingManager::save_data(){
    logger_.save_log_data();
}


} // end namespace labrob
