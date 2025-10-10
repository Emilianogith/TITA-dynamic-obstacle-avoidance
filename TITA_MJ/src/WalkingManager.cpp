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

  

    // auto q_init = robot_state_to_pinocchio_joint_configuration(
    //     robot_model_,
    //     initial_robot_state
    // );
    // pinocchio::forwardKinematics(robot_model_, robot_data_, q_init);
    // pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q_init);
    // pinocchio::framesForwardKinematics(robot_model_, robot_data_, q_init);
    


    int njnt = robot_model_.nv - 6;

    // TODO: init using node handle.
    controller_frequency_ = 100;                        // CONTROLLA!!!!!!
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
    des_configuration_.lwheel_contact.pos.p = Eigen::Vector3d(0.0, 0.28, 0.095);
    des_configuration_.lwheel_contact.pos.R = Eigen::Matrix3d::Identity();     // desired orientation of the contact frame
    // des_configuration_.lwheel_contact.pos.R << 1,0,0,  0, 0.984,0.1736,  -0.1736,0.984,0;
    des_configuration_.lwheel_contact.vel = Eigen::Vector<double, 6>::Zero();
    des_configuration_.lwheel_contact.acc = Eigen::Vector<double, 6>::Zero();
    des_configuration_.rwheel_contact.pos.p = Eigen::Vector3d(0.0, -0.28, 0.095);
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
    Eigen::Vector3d p_CoM = robot_data_.com[0];
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
    


    auto params = WholeBodyControllerParams::getDefaultParams();
    // params.Kp_motion = 13000;                   
    // params.Kd_motion = 300;                    
    // params.Kp_regulation = 1000.0;            
    // params.Kd_regulation = 50;                 

    // params.Kp_wheel = 60000.0;                 
    // params.Kd_wheel = 150.0;                 

    // params.weight_q_ddot = 1e-4;                
    // params.weight_com = 1.1;                    
    // params.weight_lwheel = 2.0;                 
    // params.weight_rwheel = 2.0;                 
    // params.weight_base = 0.05;                  
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
    // mpc_timings_log_file_.open("/tmp/mpc_timings.txt");
    mpc_com_log_file_.open("/tmp/mpc_com.txt");
    mpc_zmp_log_file_.open("/tmp/mpc_zmp.txt");
    com_log_file_.open("/tmp/com.txt");
    zmp_log_file_.open("/tmp/zmp.txt");

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
    // pinocchio::framesForwardKinematics(robot_model_, robot_data_, q);
    pinocchio::centerOfMass(robot_model_, robot_data_, q, qdot, 0.0 * qdot); // This is used to compute the CoM drift (J_com_dot * qdot)
    const auto& centroidal_momentum_matrix = pinocchio::ccrba(
        robot_model_,
        robot_data_,
        q,
        qdot
    );
    // auto angular_momentum = (centroidal_momentum_matrix * qdot).tail<3>();

    const auto& p_CoM = robot_data_.com[0];
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

   

    // LQR
    double g = 9.81;
    double z0 = des_configuration_.com.pos(2);            // CoM height [m]
    double eta = std::sqrt(g / z0);            // sqrt(g/z0)
    double Ts = 0.01;                           // sampling time [s]
    int N = 100;                                // horizon length

    const auto& v_CoM = robot_data_.vcom[0];

    // Exact discretization
    const double s = std::sinh(eta*Ts), c = std::cosh(eta*Ts);
    Eigen::Matrix2d A;
    A << c, s/eta,
        eta*s, c;
    Eigen::Vector2d B;
    B << 1 - c,
        -eta*s;

    // --- LQR weights
    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0,0) = 2250.0;           // pos weight                         
    Q(1,1) = 2900.0;           // vel weight      
    
    Eigen::Matrix<double,1,1> R;
    R(0,0) = 2620;       // input (ZMP) weight                

    Eigen::Matrix2d Qf = Eigen::Matrix2d::Zero();   
    Qf(0,0) = 2250.0;             // pos weight
    Qf(1,1) = 2900.0;             // vel weight

    // --- Storage for TV-LQR
    std::vector<Eigen::Matrix2d> P(N+1);
    std::vector<Eigen::RowVector2d> K(N); 

    // Terminal condition
    P[N] = Qf;

    // Backward Riccati (time-varying LQR)
    for (int k = N-1; k >= 0; --k) {
        // S = R + B^T P_{k+1} B  (scalar here)
        double S = (R + (B.transpose() * P[k+1] * B))(0,0);
        // K_k = S^{-1} B^T P_{k+1} A   (1x2)
        K[k] = (1.0 / S) * (B.transpose() * P[k+1] * A);
        // P_k = Q + A^T (P_{k+1} - P_{k+1} B S^{-1} B^T P_{k+1}) A
        Eigen::Matrix2d term = P[k+1] - (P[k+1] * B) * (1.0 / S) * (B.transpose() * P[k+1]);
        P[k] = Q + A.transpose() * term * A;
    }

    // --- Reference tracking (constant setpoint c*)
    double c_star = 0.0;
    Eigen::Vector2d x_ref(c_star, 0.0);
    double u_ref = c_star;   // steady-state ZMP = c*

    // --- Closed-loop rollout over the horizon (predict)
    Eigen::VectorXd x_ZMP_des  = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd v_ZMP_des  = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd a_ZMP_des  = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd x_CoM_des = Eigen::VectorXd::Zero(N+1);
    Eigen::VectorXd v_CoM_des = Eigen::VectorXd::Zero(N+1);
    Eigen::VectorXd a_CoM_des  = Eigen::VectorXd::Zero(N+1);

    // Current state (measurements)
    double c_now = p_CoM(0);
    double cdot_now = v_CoM(0);
    Eigen::Vector2d x(c_now, cdot_now);

    x_CoM_des(0) = x(0);
    v_CoM_des(0) = x(1);
    a_CoM_des(0) = robot_data_.acom[0](0);
    
    double u_prev = des_configuration_.lwheel_contact.pos.p(0);
    double v_prev = des_configuration_.lwheel_contact.vel(0);
    double a_prev = des_configuration_.lwheel_contact.acc(0);

    for (int k = 0; k < N; ++k) {
        Eigen::Vector2d e = x - x_ref;
        double u = u_ref - (K[k] * e)(0,0);     // optimal input at stage k
        // apply / predict
        x = A * x + B * u;
        x_ZMP_des(k) = u;
        
        x_CoM_des(k+1) = x(0);
        v_CoM_des(k+1) = x(1);
        a_CoM_des(k+1) = eta*eta* (x(0) - u);

        // vel, acc derivation
        const double vmax=1.0, amax=3.0, jmax=20.0; // tune
        double u_rate = std::clamp((u - u_prev)/Ts, -vmax, vmax);
        double u_lim  = u_prev + u_rate*Ts;

        double v_cmd  = (u_lim - u_prev)/Ts;
        double dv     = std::clamp(v_cmd - v_prev, -amax*Ts, amax*Ts);
        double v_lim  = v_prev + dv;

        double a_cmd  = (v_lim - v_prev)/Ts;
        double da     = std::clamp(a_cmd - a_prev, -jmax*Ts, jmax*Ts);
        double a_lim  = a_prev + da;

        x_ZMP_des(k) = u_lim;  v_ZMP_des(k) = v_lim;  a_ZMP_des(k) = a_lim;
        u_prev = u_lim;  v_prev = v_lim;  a_prev = a_lim;
    }


    des_configuration_.com.pos(0) = x_CoM_des(1);  
    des_configuration_.com.vel(0) = v_CoM_des(1);
    des_configuration_.com.acc(0) = a_CoM_des(1); 

    des_configuration_.lwheel_contact.pos.p(0) = x_ZMP_des(0);
    des_configuration_.lwheel_contact.vel(0) = v_ZMP_des(0);
    des_configuration_.lwheel_contact.acc(0) = a_ZMP_des(0);

    des_configuration_.rwheel_contact.pos.p(0) = x_ZMP_des(0);
    des_configuration_.rwheel_contact.vel(0) = v_ZMP_des(0);
    des_configuration_.rwheel_contact.acc(0) = a_ZMP_des(0);


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
    if (std::fabs(t_msec_ - 9360.0) < 0.5){
        mpc_com_log_file_ << "t_msec_: " << t_msec_ << std::endl << x_CoM_des.transpose() << std::endl;
        mpc_zmp_log_file_ << "t_msec_: " << t_msec_ << std::endl << x_ZMP_des.transpose() << std::endl;
    }

    com_log_file_ << p_CoM.transpose() << std::endl;
    zmp_log_file_ << x_ZMP_des(0) << std::endl;
}


} // end namespace labrob
