#include <WholeBodyController.hpp>


namespace labrob {

WholeBodyControllerParams WholeBodyControllerParams::getDefaultParams() {
  static WholeBodyControllerParams params;

  params.Kp_motion = 13000;                   //13000.0; 
  params.Kd_motion = 300;                     //300.0;   
  params.Kp_regulation = 1000.0;              //1000.0; 
  params.Kd_regulation = 50;                  //50.0;

  params.Kp_wheel = 90000.0;                  //60000.0;
  params.Kd_wheel = 200.0;                    //150.0;

  params.weight_q_ddot = 1e-4;                //1e-4;
  params.weight_com = 1.0;                    //1.0;
  params.weight_lwheel = 2.0;                 // 2.0;
  params.weight_rwheel = 2.0;                 // 2.0;
  params.weight_base = 0.1;                  //0.05; 
  params.weight_angular_momentum = 0.0001;    //0.0001;
  params.weight_regulation = 0.0;             // 1e-4  

  params.cmm_selection_matrix_x = 1e-6;       //1e-6;
  params.cmm_selection_matrix_y = 1e-6;       //1e-6;
  params.cmm_selection_matrix_z = 1e-4;       //1e-4;

  params.gamma = 10;                          //10;
  params.mu = 0.5;                            //0.5;

  return params;
}

WholeBodyController::WholeBodyController(
    const WholeBodyControllerParams& params,
    const pinocchio::Model& robot_model,
    const labrob::RobotState& initial_robot_state,
    double sample_time,
    std::map<std::string, double>& armatures): 
    robot_model_(robot_model),
    sample_time_(sample_time),
    params_(params)
{

  robot_data_ = pinocchio::Data(robot_model_);

  right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
  left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
  base_link_idx_ = robot_model_.getFrameId("base_link");

  wheel_radius_ = 0.0925;
  wheel_width_  = 0.017;

  J_right_wheel_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_left_wheel_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_base_link_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  J_right_wheel_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_left_wheel_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_base_link_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  n_joints_ = robot_model_.nv - 6;
  n_contacts_ = 2;
  n_wbc_variables_ = 6 + n_joints_ + 2 * 3 * n_contacts_;
  n_wbc_equalities_ = 6 + 2 * 3 + 2 * 3 * n_contacts_;
  n_wbc_inequalities_ = 2 * 4 * n_contacts_+ 2 * n_joints_;

  M_armature_ = Eigen::VectorXd::Zero(n_joints_);
  for (pinocchio::JointIndex joint_id = 2;
       joint_id < (pinocchio::JointIndex) robot_model_.njoints;
       ++joint_id) {
    std::string joint_name = robot_model_.names[joint_id];
    M_armature_(joint_id - 2) = armatures[joint_name];
  }

  wbc_solver_ptr_ = std::make_unique<qpsolvers::QPSolverEigenWrapper<double>>(
      std::make_shared<qpsolvers::HPIPMQPSolver>(
          n_wbc_variables_, n_wbc_equalities_, n_wbc_inequalities_
      )
  );
}

labrob::JointCommand
WholeBodyController::compute_inverse_dynamics(
    const labrob::RobotState& robot_state,
    const labrob::DesiredConfiguration& desired
) {

  auto start_time = std::chrono::high_resolution_clock::now();
  
  auto q = robot_state_to_pinocchio_joint_configuration(robot_model_, robot_state);
  auto qdot = robot_state_to_pinocchio_joint_velocity(robot_model_, robot_state);
  
  // Compute pinocchio terms
  pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q);
  pinocchio::computeJointJacobiansTimeVariation(robot_model_, robot_data_, q, qdot);
  pinocchio::framesForwardKinematics(robot_model_, robot_data_, q);

  pinocchio::getFrameJacobian(robot_model_, robot_data_, base_link_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_base_link_);
  pinocchio::getFrameJacobian(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_);
  pinocchio::getFrameJacobian(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_);

  pinocchio::centerOfMass(robot_model_, robot_data_, q, qdot, 0.0 * qdot); // This is to compute the drift term
  pinocchio::getFrameJacobianTimeVariation(robot_model_, robot_data_, base_link_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_base_link_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_dot_);
  
  // J_contact_wheel construction
  Eigen::Vector3d r_wheel_center = robot_data_.oMf[right_leg4_idx_].translation();
  Eigen::Matrix3d r_wheel_R = robot_data_.oMf[right_leg4_idx_].rotation();
  Eigen::Vector3d l_wheel_center = robot_data_.oMf[left_leg4_idx_].translation();
  Eigen::Matrix3d l_wheel_R = robot_data_.oMf[left_leg4_idx_].rotation();

  // J_right_contact
  Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center, r_wheel_R, wheel_radius_);
  Eigen::MatrixXd J_right_contact_v(3, robot_model_.nv);
  J_right_contact_v = J_right_wheel_.topRows(3) - pinocchio::skew(right_rCP) * J_right_wheel_.bottomRows(3);
  Eigen::Vector3d w_r = J_right_wheel_.bottomRows<3>() * qdot;
  Eigen::MatrixXd J_right_contact_v_dot(3, robot_model_.nv);
  J_right_contact_v_dot = J_right_wheel_dot_.topRows(3) - pinocchio::skew(w_r.cross(right_rCP)) * J_right_wheel_.bottomRows(3)- pinocchio::skew(right_rCP) * J_right_wheel_dot_.bottomRows(3);
  
  Eigen::MatrixXd J_right_contact(6, robot_model_.nv); 
  J_right_contact.topRows(3) = J_right_contact_v;
  J_right_contact.bottomRows(3) = J_right_wheel_.bottomRows<3>();
  Eigen::MatrixXd J_right_contact_dot(6, robot_model_.nv);
  J_right_contact_dot.topRows(3) = J_right_contact_v_dot;
  J_right_contact_dot.bottomRows(3) = J_right_wheel_dot_.bottomRows<3>();
  
  // J_left_contact
  Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center, l_wheel_R, wheel_radius_);
  Eigen::MatrixXd J_left_contact_v(3, robot_model_.nv);
  J_left_contact_v = J_left_wheel_.topRows(3) - pinocchio::skew(left_rCP) * J_left_wheel_.bottomRows(3);
  Eigen::Vector3d w_l = J_left_wheel_.bottomRows<3>() * qdot;
  Eigen::MatrixXd J_left_contact_v_dot(3, robot_model_.nv);
  J_left_contact_v_dot = J_left_wheel_dot_.topRows(3) - pinocchio::skew(w_l.cross(left_rCP)) * J_left_wheel_.bottomRows(3)- pinocchio::skew(left_rCP) * J_left_wheel_dot_.bottomRows(3);
 
  Eigen::MatrixXd J_left_contact(6, robot_model_.nv);
  J_left_contact.topRows(3) = J_left_contact_v;
  J_left_contact.bottomRows(3) = J_left_wheel_.bottomRows<3>();
  Eigen::MatrixXd J_left_contact_dot(6, robot_model_.nv);
  J_left_contact_dot.topRows(3) = J_left_contact_v_dot;
  J_left_contact_dot.bottomRows(3) = J_left_wheel_dot_.bottomRows<3>();

  Eigen::Vector3d right_contact = r_wheel_center + right_rCP;
  Eigen::Vector3d left_contact = l_wheel_center + left_rCP;



  const auto& J_com = robot_data_.Jcom;
  const auto& centroidal_momentum_matrix = pinocchio::ccrba(robot_model_, robot_data_, q, qdot);
  const auto& a_com_drift = robot_data_.acom[0];
  const auto a_lwheel_drift = J_left_wheel_dot_ * qdot;
  const auto a_rwheel_drift = J_right_wheel_dot_ * qdot;
  const auto a_base_orientation_drift = J_base_link_dot_.bottomRows<3>() * qdot;
  
  Eigen::Vector3d current_com_pos = robot_data_.com[0];
  Eigen::Vector3d current_com_vel = robot_data_.vcom[0];

  Eigen::Matrix3d current_base_link_pos = robot_data_.oMf[base_link_idx_].rotation();
  Eigen::Vector3d current_base_link_vel = J_base_link_.bottomRows<3>() * qdot;

  Eigen::MatrixXd r_contact_frame_R = labrob::compute_contact_frame(r_wheel_center, r_wheel_R, wheel_radius_);
  Eigen::MatrixXd l_contact_frame_R = labrob::compute_contact_frame(l_wheel_center, l_wheel_R, wheel_radius_);
  labrob::SE3 current_lwheel_contact_pos = labrob::SE3(l_contact_frame_R, l_wheel_center);
  Eigen::Vector<double, 6> current_lwheel_contact_vel = J_left_wheel_ * qdot;

  labrob::SE3 current_rwheel_contact_pos = labrob::SE3(r_contact_frame_R, r_wheel_center);
  Eigen::Vector<double, 6> current_rwheel_contact_vel = J_right_wheel_ * qdot;


  // Compute desired accelerations
  auto err_com = desired.com.pos - current_com_pos;
  auto err_com_vel = desired.com.vel - current_com_vel;

  auto err_lwheel = err_frameplacement(
      pinocchio::SE3(desired.lwheel_contact.pos.R, desired.lwheel_contact.pos.p), 
      pinocchio::SE3(current_lwheel_contact_pos.R, current_lwheel_contact_pos.p)
  );
  auto err_lwheel_vel = desired.lwheel_contact.vel - current_lwheel_contact_vel;

  auto err_rwheel = err_frameplacement(
      pinocchio::SE3(desired.rwheel_contact.pos.R, desired.rwheel_contact.pos.p),    
      pinocchio::SE3(current_rwheel_contact_pos.R, current_rwheel_contact_pos.p)
  );

  auto err_rwheel_vel = desired.rwheel_contact.vel - current_rwheel_contact_vel;

  auto err_base_orientation = err_rotation(desired.base_link.pos, current_base_link_pos);
  auto err_base_orientation_vel = desired.base_link.vel - current_base_link_vel;

  Eigen::VectorXd q_joint = q.tail(q.size() - 7);
  Eigen::VectorXd qdot_joint = qdot.tail(qdot.size() - 6);

  Eigen::VectorXd err_posture(6 + n_joints_);
  err_posture << Eigen::VectorXd::Zero(6), desired.qjnt - q_joint;

  Eigen::VectorXd err_posture_vel(6 + n_joints_); 
  err_posture_vel << Eigen::VectorXd::Zero(6), desired.qjntdot - qdot_joint;

  Eigen::MatrixXd err_posture_selection_matrix = Eigen::MatrixXd::Zero(6 + n_joints_, 6 + n_joints_);
  Eigen::MatrixXd matrix_with_no_wheel = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
  matrix_with_no_wheel(3, 3) -= 1;
  matrix_with_no_wheel(7, 7) -= 1;
  err_posture_selection_matrix.block(6, 6, n_joints_, n_joints_) = matrix_with_no_wheel;

  Eigen::MatrixXd cmm_selection_matrix = Eigen::MatrixXd::Zero(3, 6);
  cmm_selection_matrix(0, 3) = params_.cmm_selection_matrix_x;
  cmm_selection_matrix(1, 4) = params_.cmm_selection_matrix_y;
  cmm_selection_matrix(2, 5) = params_.cmm_selection_matrix_z;

  Eigen::VectorXd desired_qddot(6 + n_joints_);
  desired_qddot << Eigen::VectorXd::Zero(6), desired.qjntddot;
  Eigen::VectorXd a_jnt_total = desired_qddot + params_.Kp_regulation * err_posture + params_.Kd_regulation * err_posture_vel;
  Eigen::VectorXd a_com_total = desired.com.acc + params_.Kp_motion * err_com + params_.Kd_motion * err_com_vel;
  Eigen::VectorXd a_lwheel_total = desired.lwheel_contact.acc + params_.Kp_wheel * err_lwheel + params_.Kd_wheel * err_lwheel_vel;
  Eigen::VectorXd a_rwheel_total = desired.rwheel_contact.acc + params_.Kp_wheel * err_rwheel + params_.Kd_wheel * err_rwheel_vel;
  Eigen::VectorXd a_base_orientation_total = desired.base_link.acc + params_.Kp_motion * err_base_orientation + params_.Kd_motion * err_base_orientation_vel;
  

  // Build cost function
  Eigen::MatrixXd H_acc = Eigen::MatrixXd::Zero(6 + n_joints_, 6 + n_joints_);
  Eigen::VectorXd f_acc = Eigen::VectorXd::Zero(6 + n_joints_);

  H_acc += params_.weight_q_ddot * Eigen::MatrixXd::Identity(6 + n_joints_, 6 + n_joints_);
  H_acc += params_.weight_com * (J_com.transpose() * J_com);
  H_acc += params_.weight_lwheel * (J_left_wheel_.transpose() * J_left_wheel_);
  H_acc += params_.weight_rwheel * (J_right_wheel_.transpose() * J_right_wheel_);
  H_acc += params_.weight_base * (J_base_link_.bottomRows<3>().transpose() * J_base_link_.bottomRows<3>());
  H_acc += params_.weight_regulation * err_posture_selection_matrix;
  H_acc += params_.weight_angular_momentum * centroidal_momentum_matrix.transpose() * cmm_selection_matrix.transpose() *
      std::pow(sample_time_, 2.0) * cmm_selection_matrix * centroidal_momentum_matrix;

  f_acc += params_.weight_com * J_com.transpose() * (a_com_drift - a_com_total);
  f_acc += params_.weight_lwheel * J_left_wheel_.transpose() * (a_lwheel_drift - a_lwheel_total);
  f_acc += params_.weight_rwheel * J_right_wheel_.transpose() * (a_rwheel_drift - a_rwheel_total);
  f_acc += params_.weight_base * J_base_link_.bottomRows<3>().transpose() * (a_base_orientation_drift - a_base_orientation_total);
  f_acc += -params_.weight_regulation * err_posture_selection_matrix * a_jnt_total;
  f_acc += params_.weight_angular_momentum * centroidal_momentum_matrix.transpose() * cmm_selection_matrix.transpose() *
      sample_time_ * cmm_selection_matrix * centroidal_momentum_matrix * qdot;


  //Joint limits constraints
  auto q_jnt_dot_min = -robot_model_.velocityLimit.tail(n_joints_);
  auto q_jnt_dot_max = robot_model_.velocityLimit.tail(n_joints_);
  auto q_jnt_min = robot_model_.lowerPositionLimit.tail(n_joints_);
  auto q_jnt_max = robot_model_.upperPositionLimit.tail(n_joints_);

  auto tau_min = -robot_model_.effortLimit.tail(n_joints_);
  auto tau_max = robot_model_.effortLimit.tail(n_joints_);

  Eigen::MatrixXd C_acc = Eigen::MatrixXd::Zero(2 * n_joints_, 6 + n_joints_);
  Eigen::VectorXd d_min_acc(2 * n_joints_);
  Eigen::VectorXd d_max_acc(2 * n_joints_);
  C_acc.rightCols(n_joints_).topRows(n_joints_).diagonal().setConstant(sample_time_);
  C_acc.rightCols(n_joints_).bottomRows(n_joints_).diagonal().setConstant(std::pow(sample_time_, 2.0) / 2.0);
  d_min_acc << q_jnt_dot_min - qdot_joint, q_jnt_min - q_joint - sample_time_ * qdot_joint;
  d_max_acc << q_jnt_dot_max - qdot_joint, q_jnt_max - q_joint - sample_time_ * qdot_joint;


  Eigen::MatrixXd M = pinocchio::crba(robot_model_, robot_data_, q);
  // We need to do this since the inertia matrix in Pinocchio is only upper triangular
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  M.diagonal().tail(n_joints_) += M_armature_;

  // Computing Coriolis, centrifugal and gravitational effects
  auto& c = pinocchio::rnea(robot_model_, robot_data_, q, qdot, Eigen::VectorXd::Zero(6 + n_joints_));

  Eigen::MatrixXd Jlu = J_left_contact.block(0,0,6,6);                
  Eigen::MatrixXd Jla = J_left_contact.block(0,6,6,n_joints_);
  Eigen::MatrixXd Jru = J_right_contact.block(0,0,6,6);
  Eigen::MatrixXd Jra = J_right_contact.block(0,6,6,n_joints_);

  Eigen::MatrixXd Mu = M.block(0,0,6,6+n_joints_);                // fb + n_joints
  Eigen::MatrixXd Ma = M.block(6,0,n_joints_,6+n_joints_);        // n_joints

  Eigen::VectorXd cu = c.block(0,0,6,1);
  Eigen::VectorXd ca = c.block(6,0,n_joints_,1);

  Eigen::MatrixXd T_l(6, 3 * n_contacts_);
  Eigen::MatrixXd T_r(6, 3 * n_contacts_);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  if (n_contacts_ == 1){
    std::vector<Eigen::Vector3d> pcis(1);
    pcis[0] <<  0.0,  0.0, 0.0;
  
    std::vector<Eigen::Vector3d> pcis_l(1);
    std::vector<Eigen::Vector3d> pcis_r(1);

    for (int i = 0; i < n_contacts_; ++i) {
    pcis_l[i] = desired.lwheel_contact.pos.R * pcis[i];
    pcis_r[i] = desired.rwheel_contact.pos.R * pcis[i];
  }
    T_l << I3,
        pinocchio::skew(pcis_l[0]);
    T_r << I3,
        pinocchio::skew(pcis_r[0]);

  } else if (n_contacts_ == 2) {
    std::vector<Eigen::Vector3d> pcis(2);
    pcis[0] <<  0.0,  wheel_width_ / 2.0, 0.0;
    pcis[1] <<  0.0, -wheel_width_ / 2.0, 0.0;
  
    std::vector<Eigen::Vector3d> pcis_l(2);
    std::vector<Eigen::Vector3d> pcis_r(2);

    for (int i = 0; i < n_contacts_; ++i) {
    pcis_l[i] = desired.lwheel_contact.pos.R * pcis[i];
    pcis_r[i] = desired.rwheel_contact.pos.R * pcis[i];
  }
    T_l << I3, I3,
          pinocchio::skew(pcis_l[0]), pinocchio::skew(pcis_l[1]);
    T_r << I3, I3,
          pinocchio::skew(pcis_r[0]), pinocchio::skew(pcis_r[1]);
  }

  
  // QP formulation
  Eigen::MatrixXd H_force_one = 1e-2 * Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
  Eigen::VectorXd f_force_one = Eigen::VectorXd::Zero(3 * n_contacts_);

  Eigen::VectorXd b_dyn = -cu;

  Eigen::MatrixXd C_force_block(4, 3);
  C_force_block <<  1.0,  0.0, -params_.mu,
                    0.0,  1.0, -params_.mu,
                   -1.0,  0.0, -params_.mu,
                    0.0, -1.0, -params_.mu;
  
  Eigen::VectorXd d_min_force_one = -10000.0 * Eigen::VectorXd::Ones(4 * n_contacts_);
  Eigen::VectorXd d_max_force_one = Eigen::VectorXd::Zero(4 * n_contacts_);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(H_acc.rows() + 2 * H_force_one.rows(), H_acc.cols() + 2 * H_force_one.cols());
  H.block(0, 0, H_acc.rows(), H_acc.cols()) = H_acc;
  H.block(H_acc.rows(), H_acc.cols(), H_force_one.rows(), H_force_one.cols()) = H_force_one;
  H.block(H_acc.rows() + H_force_one.rows(),
          H_acc.cols() + H_force_one.cols(),
          H_force_one.rows(),
          H_force_one.cols()) = H_force_one;
  Eigen::VectorXd f(f_acc.size() + 2 * f_force_one.size());
  f << f_acc, f_force_one, f_force_one;

  // No contact constraint
  Eigen::MatrixXd A_no_contact = Eigen::MatrixXd::Zero(2 * 3 * n_contacts_, 2 * 3 * n_contacts_);   
  Eigen::VectorXd b_no_contact = Eigen::VectorXd::Zero(2 * 3 * n_contacts_);
  
  // if (desired.in_contact == false){
  //   Eigen::MatrixXd A_no_contact_left = Eigen::MatrixXd::Zero(3 * n_contacts_, 3 * n_contacts_);   
  //   Eigen::MatrixXd A_no_contact_right = Eigen::MatrixXd::Zero(3 * n_contacts_, 3 * n_contacts_);   
  //   A_no_contact_left = Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
  //   A_no_contact_right = Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
  //   A_no_contact.block(0,0, A_no_contact_left.rows(), A_no_contact_left.cols()) = A_no_contact_left;
  //   A_no_contact.block(A_no_contact_left.rows(), A_no_contact_left.cols(), A_no_contact_right.rows(), A_no_contact_right.cols()) = A_no_contact_right;
  // }
  
  // No slipping constraint: a_contact = J_contact * qddot + J_dot * qdot = 0 + Kd*(0 - pdot) //+ Kp*(p_d - p) wherever is the contact point
  Eigen::MatrixXd A_acc = Eigen::MatrixXd::Zero(6, 6 + n_joints_);
  Eigen::VectorXd b_acc = Eigen::VectorXd::Zero(6);
  A_acc.topRows(3) = J_left_contact.topRows(3);
  b_acc.topRows(3) = -J_left_contact_dot.topRows(3) * qdot - params_.gamma * J_left_contact.topRows(3) * qdot;

  A_acc.bottomRows(3) = J_right_contact.topRows(3);  
  b_acc.bottomRows(3) = -J_right_contact_dot.topRows(3) * qdot - params_.gamma * J_right_contact.topRows(3) * qdot;

  // if (desired.in_contact == false){
  //   A_acc *= 0.0; // REMOVED for deubg
  //   b_acc *= 0.0;
  // }

  // Floating base no actuation constraint
  Eigen::MatrixXd A_dyn(6, 6 + n_joints_ + 2 * 3 * n_contacts_);
  A_dyn << Mu, -Jlu.transpose() * T_l, -Jru.transpose() * T_r;
  

  // Build A and b matrices
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_acc.rows() + A_no_contact.rows() + A_dyn.rows(), n_wbc_variables_);
  A.block(0, 0, A_acc.rows(), A_acc.cols()) = A_acc;
  A.block(A_acc.rows(), A_acc.cols(), A_no_contact.rows(), A_no_contact.cols()) = A_no_contact;
  A.bottomRows(A_dyn.rows()) = A_dyn;
  Eigen::VectorXd b(b_acc.rows() + b_no_contact.rows() + b_dyn.rows());
  b << b_acc, b_no_contact, b_dyn;
  
  
  // Contatct forces constraints
  Eigen::MatrixXd C_force_left = Eigen::MatrixXd::Zero(4 * n_contacts_, 3 * n_contacts_);
  for (int i = 0; i < n_contacts_; ++i) {
    C_force_left.block(4 * i, 3 * i, 4, 3) = C_force_block * current_lwheel_contact_pos.R.transpose();
  }
  Eigen::MatrixXd C_force_right = Eigen::MatrixXd::Zero(4 * n_contacts_, 3 * n_contacts_);
  for (int i = 0; i < n_contacts_; ++i) {
    C_force_right.block(4 * i, 3 * i, 4, 3) = C_force_block * current_rwheel_contact_pos.R.transpose();
  }

  // Build C and d matrices
  Eigen::MatrixXd C(C_acc.rows() + 2 * C_force_left.rows(), n_wbc_variables_);
  C << C_acc, Eigen::MatrixXd::Zero(C_acc.rows(), 2 * 3 * n_contacts_),
      Eigen::MatrixXd::Zero(C_force_left.rows(), 6 + n_joints_), C_force_left, Eigen::MatrixXd::Zero(C_force_left.rows(), 3 * n_contacts_),
      Eigen::MatrixXd::Zero(C_force_right.rows(), 6 + n_joints_), Eigen::MatrixXd::Zero(C_force_right.rows(), 3 * n_contacts_), C_force_right;
  Eigen::VectorXd d_min(d_min_acc.rows() + 2 * d_min_force_one.rows());
  Eigen::VectorXd d_max(d_max_acc.rows() + 2 * d_max_force_one.rows());
  d_min << d_min_acc, d_min_force_one, d_min_force_one;
  d_max << d_max_acc, d_max_force_one, d_max_force_one;

  
  

  // DEBUG PRINT
  // std::cout << " r_wheel_center" << r_wheel_center << std::endl;
  // std::cout << " l_wheel_center" << l_wheel_center << std::endl;
  // std::cout << " current_com_pos" << current_com_pos << std::endl;

  // std::cout << "err_com" << err_com << std::endl;
  // std::cout << "err_rwheel" << err_rwheel << std::endl;
  // std::cout << "err_lwheel" << err_lwheel << std::endl;
  // std::cout << "err_posture" << err_posture << std::endl;



  wbc_solver_ptr_->solve(H, f, A, b, C, d_min, d_max);
  Eigen::VectorXd solution = wbc_solver_ptr_->get_solution();
  Eigen::VectorXd q_ddot = solution.head(6 + n_joints_);
  Eigen::VectorXd flr = solution.tail(2 * 3 * n_contacts_);
  Eigen::VectorXd fl = flr.head(3 * n_contacts_);
  Eigen::VectorXd fr = flr.tail(3 * n_contacts_);
  Eigen::VectorXd tau = Ma * q_ddot + ca - Jla.transpose() * T_l * fl - Jra.transpose() * T_r * fr;  // * T_l,  *T_r

  
  // std::cout << "computed tau" << tau << std::endl;
  // std::cout << "computed solution" << solution << std::endl;


  // Eigen::VectorXd r_val = J_right_contact_v * q_ddot + J_right_contact_v_dot * qdot + params_.gamma * J_right_contact_v * qdot;
  // Eigen::VectorXd l_val = J_left_contact_v * q_ddot + J_left_contact_v_dot * qdot + params_.gamma * J_left_contact_v * qdot;

  // if (r_val.norm() >1e-9){
  //   std::cout << "acc P contact right: " << J_right_contact_v * q_ddot + J_right_contact_v_dot * qdot + params_.gamma * J_right_contact_v * qdot << std::endl;
  // }
  //  if (l_val.norm() >1e-9){
  //    std::cout << "acc P contact left: " << J_left_contact_v * q_ddot + J_left_contact_v_dot * qdot + params_.gamma * J_left_contact_v * qdot  << std::endl;
  // }


  // Fine misurazione del tempo
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // Stampa del tempo di esecuzione
  std::cout << "Tempo di esecuzione del controllore Whole Body: " << duration << " microsecondi" << std::endl;


  JointCommand joint_command;
  for(pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex) robot_model_.njoints; ++joint_id) {
    const auto& joint_name = robot_model_.names[joint_id];
    joint_command[joint_name] = tau[joint_id - 2];
  }
  
  return joint_command;
}

}