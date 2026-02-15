#include <WalkingManager.hpp>


namespace labrob {

bool WalkingManager::init(const labrob::RobotState& initial_robot_state,
                     std::map<std::string, double> &armatures,
                     const pinocchio::Model& robot_model) {
    
    robot_model_ = &robot_model;
    robot_data_ = pinocchio::Data(*robot_model_);
    
    right_leg4_idx_ = robot_model_->getFrameId("right_leg_4");
    left_leg4_idx_ = robot_model_->getFrameId("left_leg_4");

    int njnt = robot_model_->nv - 6;

    // TODO: init using node handle.
    controller_frequency_ = 500;                                    // nominal control frequency 
    controller_timestep_msec_ = 1000 / controller_frequency_;
    

    wheel_radius_ = 0.0925;
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
    des_configuration_.base_link.pos =Eigen::Matrix3d::Identity();
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
    auto q = robot_state_to_pinocchio_joint_configuration(*robot_model_, initial_robot_state);
    auto qdot = robot_state_to_pinocchio_joint_velocity(*robot_model_, initial_robot_state);

    pinocchio::centerOfMass(*robot_model_, robot_data_, q, qdot);      // compute com pos and vel
    pinocchio::framesForwardKinematics(*robot_model_, robot_data_, q); // update robot_data_.oMf
    pinocchio::computeJointJacobians(*robot_model_, robot_data_, q);   // compute joint jacobians


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

    // plan the offline trajectory
    walkingPlanner_.offline_plan(p_CoM, 0.001 * controller_timestep_msec_, true);

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


    // Init log files:
    // TODO: may be better to use a proper logging system such as glog.
    // state_log_file_.open("/tmp/state_log_file.txt");
    // state_log_file_ << "time,"
    //      << "com_x,com_y,com_z,"
    //      << "com_x_des,com_y_des,com_z_des,"
    //      << "wheel_l_x,wheel_l_y,wheel_l_z,"
    //      << "wheel_l_x_des,wheel_l_y_des,wheel_l_z_des,"
    //      << "wheel_r_x,wheel_r_y,wheel_r_z,"
    //      << "wheel_r_x_des,wheel_r_y_des,wheel_r_z_des"
    //      << std::endl;

    return true;
    }



void WalkingManager::update(
    const labrob::RobotState& robot_state,
    labrob::JointCommand& joint_torque, 
    labrob::JointCommand& joint_acceleration,
    double t_msec_) {

    // auto start_time = std::chrono::system_clock::now();

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


    // jump routine
    // if (std::fabs(t_msec_ - 2000.0) < 0.5){
    //     walkingPlanner_.jumpRoutine(t_msec_);
    // }

    mpc_.t_msec = t_msec_;

    // log mpc logs
    // if (static_cast<int>(t_msec_) % 10 == 0){
    //     mpc_.record_logs = true;
    // }


    // DFIP (DDP) - based MPC
    Eigen::VectorXd x_IN(18);
    x_IN.segment<3>(0) = p_CoM;
    x_IN.segment<3>(3) = v_CoM;
    x_IN.segment<3>(6) = left_contact;
    x_IN.segment<3>(9) = right_contact;
    x_IN.segment<3>(12) = curr_pl_vel;
    x_IN.segment<3>(15) = curr_pr_vel;

    
    // auto t1 = std::chrono::system_clock::now();
    mpc_.solve(x_IN);
    // auto t2 = std::chrono::system_clock::now();
    // auto delta_t = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    // std::cout << "MPC took " << delta_t << " us" << std::endl;

    SolutionMPC sol = mpc_.get_solution();
    
    des_configuration_.com.pos = sol.com.pos;
    des_configuration_.com.vel = sol.com.vel;
    des_configuration_.com.acc = sol.com.acc;

    des_configuration_.lwheel.pos.p.segment<2>(0) = sol.pl.pos.segment<2>(0);
    des_configuration_.lwheel.pos.p(2) = sol.pl.pos(2) + wheel_radius_;             // z of the wheel center is distanciated of wheel radius from the contact (in the model of MPC which does not provide camber motion)
    des_configuration_.lwheel.vel.segment<3>(0) = sol.pl.vel.segment<3>(0);
    des_configuration_.lwheel.acc.segment<3>(0) = sol.pl.acc.segment<3>(0);

    des_configuration_.rwheel.pos.p.segment<2>(0) = sol.pr.pos.segment<2>(0);
    des_configuration_.rwheel.pos.p(2) = sol.pl.pos(2) + wheel_radius_;             // z of the wheel center is distanciated of wheel radius from the contact 
    des_configuration_.rwheel.vel.segment<3>(0) = sol.pr.vel.segment<3>(0);
    des_configuration_.rwheel.acc.segment<3>(0) = sol.pr.acc.segment<3>(0);

    Eigen::Matrix3d R_theta = Eigen::Matrix3d::Zero();
    R_theta << cos(sol.theta), -sin(sol.theta), 0,
            sin(sol.theta), cos(sol.theta), 0,
            0,0,1;
    des_configuration_.base_link.pos = R_theta;
    des_configuration_.base_link.vel = Eigen::Vector3d(0,0,sol.omega);
    des_configuration_.base_link.acc = Eigen::Vector3d(0,0,sol.alpha);

    des_configuration_.in_contact = 1 - mpc_.get_jumpingState();

    // change WBC params in jump state
    if (!des_configuration_.in_contact) {
        auto jump_params = WholeBodyControllerParams::getDefaultParams();
        jump_params.Kp_motion = 65.0;
        jump_params.Kd_motion = 40.0;   
                   
        jump_params.Kp_wheel = 65.0;       
        jump_params.Kd_wheel = 40.0;                 

        jump_params.weight_q_ddot = 1e-6;                
        jump_params.weight_com = 0.05;                     
        jump_params.weight_lwheel = 0.05;                 
        jump_params.weight_rwheel = 0.05;                 
        jump_params.weight_base = 0.01;              
        jump_params.weight_angular_momentum = 0.0;   
        jump_params.weight_regulation = 0.0; 

        jump_params.cmm_selection_matrix_x = 1e-6;       
        jump_params.cmm_selection_matrix_y = 1e-6;       
        jump_params.cmm_selection_matrix_z = 1e-4;
                                            
        whole_body_controller_ptr_->params_ = jump_params;
    } else {                                                        // TODO: avoid updating the params every cycle 
        auto params = WholeBodyControllerParams::getDefaultParams();
        whole_body_controller_ptr_->params_ = params;     
    }

    // prova regulation --------------------
    // auto params = WholeBodyControllerParams::getDefaultParams();
    // params.Kp_motion = 0.0;               
    // params.Kd_motion = 0.0;               
    // params.Kp_regulation = 0.8;            
    // params.Kd_regulation = 0.2;      

    // params.Kp_wheel = 0.0;                  
    // params.Kd_wheel = 0.0;                         

    // params.weight_q_ddot = 1e-6;                 
    // params.weight_com = 0.0;                        
    // params.weight_lwheel = 0.0;                          
    // params.weight_rwheel = 0.0;                         
    // params.weight_base = 0.0;                       
    // params.weight_angular_momentum = 0.0; 
    // params.weight_regulation = 0.1; 
    // whole_body_controller_ptr_->params_ = params;   
    
    // des_configuration_.in_contact = false;       // PER PROVA REGULATION
    // ---------------------------------------------
    
    whole_body_controller_ptr_->compute_inverse_dynamics(robot_state, des_configuration_, joint_torque, joint_acceleration);



    // auto end_time = std::chrono::system_clock::now();
    // auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    // std::cout << "WalkingManager::update() took " << elapsed_time << " us" << std::endl;
    
    // std::cout << "t_msec_ " << t_msec_ << std::endl;

    // Update timing in milliseconds.
    // NOTE: assuming update() is actually called every controller_timestep_msec_
    //       milliseconds.
    // t_msec_ += controller_timestep_msec_;


    // Log:
    // state_log_file_
    //     << t_msec_ << ","
    //     << p_CoM(0) << "," << p_CoM(1) << "," << p_CoM(2) << ","
    //     << des_configuration_.com.pos(0) << "," << des_configuration_.com.pos(1) << "," << des_configuration_.com.pos(2) << ","
    //     << l_wheel_center.translation()(0) << "," << l_wheel_center.translation()(1) << "," << l_wheel_center.translation()(2) << ","
    //     << des_configuration_.lwheel.pos.p(0) << "," << des_configuration_.lwheel.pos.p(1) << "," << des_configuration_.lwheel.pos.p(2) << ","
    //     << r_wheel_center.translation()(0) << "," << r_wheel_center.translation()(1) << "," << r_wheel_center.translation()(2) << ","
    //     << des_configuration_.rwheel.pos.p(0) << "," << des_configuration_.rwheel.pos.p(1) << "," << des_configuration_.rwheel.pos.p(2)
    //     << std::endl;
}

} // end namespace labrob
