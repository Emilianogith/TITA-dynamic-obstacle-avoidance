//
// Created by mmaximo on 20/02/24.
//

#include <WholeBodyController.hpp>

// Pinocchio
// #include <pinocchio/algorithm/centroidal.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>
// #include <pinocchio/algorithm/model.hpp>
// #include <pinocchio/algorithm/rnea.hpp>
// #include <pinocchio/algorithm/crba.hpp>
// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/algorithm/model.hpp>

// #include <JointCommand.hpp>
// #include <utils.hpp>

int count=0;

namespace labrob {

WholeBodyControllerParams WholeBodyControllerParams::getDefaultParams() {
  static WholeBodyControllerParams params;

  params.Kp_motion = 100.0;
  params.Kd_motion = 90.0;
  params.Kp_regulation = 100000.0; //100000.0
  params.Kd_regulation = 50.0;

  params.weight_q_ddot = 1e-12;
  params.weight_com = 0.0;
  params.weight_lwheel = 0.0;
  params.weight_rwheel = 0.0;
  params.weight_base = 0.0;
  params.weight_angular_momentum = 1e-4;
  params.weight_regulation = 10000.0;

  params.cmm_selection_matrix_x = 1e-6;
  params.cmm_selection_matrix_y = 1e-6;
  params.cmm_selection_matrix_z = 1e-4;

  params.gamma = params.Kd_motion;
  params.mu = 0.5;

  return params;
}

WholeBodyController::WholeBodyController(
    const labrob::RobotState& initial_robot_state,
    std::map<std::string, double>& armatures)
{

  params_ = WholeBodyControllerParams::getDefaultParams();

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


  // Init desired lsole and rsole poses:
  auto q_init = robot_state_to_pinocchio_joint_configuration(
      robot_model_,
      initial_robot_state
  );

  pinocchio::forwardKinematics(robot_model_, robot_data_, q_init);
  pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q_init);
  pinocchio::framesForwardKinematics(robot_model_, robot_data_, q_init);
  right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
  left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
  base_link_idx_ = robot_model_.getFrameId("base_link");

  // auto& T_rleg4_init_ = robot_data_.oMf[right_leg4_idx_];
  // auto& T_lleg4_init = robot_data_.oMf[left_leg4_idx_];
  
  wheel_radius_ = 0.09;
  // T_rleg4_init.translation().z() -= wheel_radius_; 
  // T_lleg4_init.translation().z() -= wheel_radius_;
   
  // Eigen::Vector3d local_offset(wheel_radius_, 0.0, 0.0);
  // pinocchio::SE3 Tc;
  // Tc.rotation() = T_rleg4_init_.rotation();                // same orientation as wheel
  // Tc.translation() = T_rleg4_init_.act(local_offset);

  // int parent_joint_id = robot_model_.frames[right_leg4_idx_].parent;  // get parent joint
  // pinocchio::Frame contact_frame("contact_C", parent_joint_id, parent_joint_id, Tc, pinocchio::FrameType::OP_FRAME);

  // // Optionally check if already inserted before adding
  // contact_frame_id_ = robot_model_.addFrame(contact_frame);


  // int wheel_frame_id = robot_model_.getFrameId("right_leg_4", pinocchio::FrameType::BODY);
  // int wheel_joint_id = robot_model_.getJointId("joint_right_leg_3");


  // frame under wheel approach
  // pinocchio::SE3 identity = pinocchio::SE3::Identity();

  // int leg3_frame_id = robot_model_.getFrameId("right_leg_3", pinocchio::FrameType::BODY);
  // int leg4_frame_id = robot_model_.getFrameId("right_leg_4", pinocchio::FrameType::BODY);
  // int leg3_joint_id = robot_model_.frames[leg3_frame_id].parentJoint;
  // // int leg3_joint_id = robot_model_.getJointId("joint_right_leg_3");
  
  // pinocchio::SE3 Tleg3_world = robot_data_.oMf[leg3_frame_id];
  // pinocchio::SE3 Tleg4_world = robot_data_.oMf[leg4_frame_id];
  // pinocchio::SE3 Tleg3_leg4 = Tleg3_world.inverse() * Tleg4_world;

  // support_frame_id_ = robot_model_.addFrame(
  //     pinocchio::Frame(
  //         "wheel_support_frame",
  //         leg3_joint_id,       // the joint connecting leg3→leg4
  //         leg3_frame_id,       // parent frame: leg3
  //         Tleg3_leg4,
  //         pinocchio::FrameType::BODY
  //     )
  // );
//   support_frame_id_ = robot_model_.addFrame(
//   pinocchio::Frame(
//     "wheel_support_frame",
//     /*parent_joint=*/ robot_model_.frames[wheel_frame_id].parentJoint,
//     /*parent_frame=*/ wheel_frame_id,
//     identity,
//     pinocchio::FrameType::BODY
//   )
// );

//   pinocchio::forwardKinematics(robot_model_, robot_data_, q_init);
//   pinocchio::jacobianCenterOfMass(robot_model_, robot_data_, q_init);
//   pinocchio::framesForwardKinematics(robot_model_, robot_data_, q_init);

  
//   pinocchio::SE3 Tsupport = robot_data_.oMf[support_frame_id_];


//   Eigen::Vector3d local_offset(wheel_radius_, 0.0, 0.0); // downwards in link frame

//   // 4. Place contact frame:
//   pinocchio::SE3 Tc;
//   Tc.rotation() = Tsupport.rotation();
//   Tc.translation() = Tsupport.act(local_offset);

//   // 5. If not present, add OP_FRAME once for contact_C
//   // (Or reuse if already created)
//   // pinocchio::Frame contact_frame(
//   //   "contact_C", wheel_joint_id, support_frame_id_, Tc,
//   //   pinocchio::FrameType::OP_FRAME
//   // );
//   pinocchio::Frame contact_frame(
//   "contact_C",
//   /*parent_joint=*/ robot_model_.frames[support_frame_id_].parentJoint,
//   /*parent_frame=*/ support_frame_id_,
//   Tc,
//   pinocchio::FrameType::OP_FRAME
// );
//   int contact_frame_id = robot_model_.addFrame(contact_frame);

//   // 6. Update its pose in data:
//   robot_data_.oMf[contact_frame_id] = Tc;

//   std::cout << "contact frame " << contact_frame_id << Tc << std::endl;

//   std::cout << "total joints" << robot_model_.njoints << std::endl;
//   for (int i = 0; i < robot_model_.njoints; ++i)
// {
//     std::cout << "Joint #" << i << ": " 
//               << robot_model_.names[i] << std::endl;
// }



    
  int njnt = robot_model_.nv - 6;   // 8
  q_jnt_reg_ = q_init.tail(njnt);
  

  int64_t controller_frequency = 600;
  sample_time_ = 0.001 * 1000 / controller_frequency;


  J_right_wheel_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_left_wheel_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_base_link_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  J_right_wheel_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_left_wheel_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
  J_base_link_dot_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);

  n_joints_ = robot_model_.nv - 6;
  n_contacts_ = 1;
  n_wbc_variables_ = 6 + n_joints_ + 2 * 3 * n_contacts_;
  n_wbc_equalities_ = 6 + 2 * 6;  //+ 3 * n_contacts_;
  n_wbc_inequalities_ = 2 * n_joints_ + 2 * 4 * n_contacts_;

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


  // non vabene devi mettere un frame nell'urdf non figlio di leg 
  // auto& T_rleg4 = robot_data_.oMf[right_leg4_idx_];
  // auto& T_lleg4 = robot_data_.oMf[left_leg4_idx_];
  
  // T_rleg4.translation().z() -= wheel_radius_; 
  // T_lleg4.translation().z() -= wheel_radius_;


  // const auto& T_wheel = robot_data_.oMf[right_leg4_idx_];
  // Eigen::Vector3d offset(wheel_radius_, 0.0, 0.0);

  // T_rleg4_init_.translation() = T_wheel.translation();

  // pinocchio::SE3 Tc = T_rleg4_init_;
  // Tc.translation() = T_rleg4_init_.act(offset);

  // // Update the contact frame in data:
  // robot_data_.oMf[contact_frame_id_] = Tc;


  // std::cout << "Tc :" << Tc << std::endl;

  // frame approach 
  // Eigen::Vector3d local_offset(wheel_radius_, 0.0, 0.0); // downwards in link frame

  // // 4. Place contact frame:
  // pinocchio::SE3 Twf = robot_data_.oMf[support_frame_id_];
  // pinocchio::SE3 Tc;


  // double c = std::cos(-q(14));
  // double s = std::sin(-q(14));

  // std::cout << "q :" << q << std::endl;
  // std::cout << "q(14) :" << q(14) << std::endl;

  // Eigen::Matrix3d Rz_negq;
  // Rz_negq <<  c,  s, 0,
  //           -s,  c, 0,
  //             0,  0, 1;

  // Twf.rotation() = Twf.rotation() * Rz_negq;
  // Tc.rotation() = Twf.rotation();
  // Tc.translation() = Twf.act(local_offset);

  // int contact_frame_id = robot_model_.getFrameId("contact_C");

  // // 6. Update its pose in data:
  // robot_data_.oMf[contact_frame_id] = Tc;

  // std::cout << "Tc :" << Tc << std::endl;



  pinocchio::getFrameJacobian(robot_model_, robot_data_, base_link_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_base_link_);
  pinocchio::getFrameJacobian(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_);
  pinocchio::getFrameJacobian(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_);



  pinocchio::centerOfMass(robot_model_, robot_data_, q, qdot, 0.0 * qdot); // This is to compute the drift term
  pinocchio::getFrameJacobianTimeVariation(robot_model_, robot_data_, base_link_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_base_link_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_dot_);
  pinocchio::getFrameJacobianTimeVariation(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_dot_);
  
  // vedi se J_contact = J_wheel()*qdot + r * qdot_wheel
  // std::cout << "J_left_wheel " << J_left_wheel_<< std::endl;



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

  labrob::SE3 current_lwheel_pos = labrob::SE3(robot_data_.oMf[left_leg4_idx_].rotation(), robot_data_.oMf[left_leg4_idx_].translation());
  Eigen::Vector<double, 6> current_lwheel_vel = J_left_wheel_ * qdot;

  labrob::SE3 current_rwheel_pos = labrob::SE3(robot_data_.oMf[right_leg4_idx_].rotation(), robot_data_.oMf[right_leg4_idx_].translation());
  Eigen::Vector<double, 6> current_rwheel_vel = J_left_wheel_ * qdot;


  // Compute desired accelerations
  auto err_com = desired.com.pos - current_com_pos;
  auto err_com_vel = desired.com.vel - current_com_vel;

  auto err_lwheel = err_frameplacement(
      pinocchio::SE3(desired.lwheel.pos.R, desired.lwheel.pos.p),
      pinocchio::SE3(current_lwheel_pos.R, current_lwheel_pos.p)
  );
  auto err_lwheel_vel = desired.lwheel.vel - current_lwheel_vel;

  auto err_rwheel = err_frameplacement(
      pinocchio::SE3(desired.rwheel.pos.R, desired.rwheel.pos.p),
      pinocchio::SE3(current_rwheel_pos.R, current_rwheel_pos.p)
  );
  auto err_rwheel_vel = desired.rwheel.vel - current_rwheel_vel;

  auto err_base_orientation = err_rotation(desired.base_link.pos, current_base_link_pos);
  auto err_base_orientation_vel = desired.base_link.vel - current_base_link_vel;

  Eigen::VectorXd q_current = q.tail(q.size() - 7);
  Eigen::VectorXd qdot_current = qdot.tail(qdot.size() - 6);

  Eigen::VectorXd err_posture(6 + n_joints_);
  err_posture << Eigen::VectorXd::Zero(6), desired.qjnt - q_current;

  Eigen::VectorXd err_posture_vel(6 + n_joints_); 
  err_posture_vel << Eigen::VectorXd::Zero(6), desired.qjntdot - qdot_current;

  Eigen::MatrixXd err_posture_selection_matrix = Eigen::MatrixXd::Zero(6 + n_joints_, 6 + n_joints_);
  err_posture_selection_matrix.block(6, 6, n_joints_, n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

  Eigen::MatrixXd cmm_selection_matrix = Eigen::MatrixXd::Zero(3, 6);
  cmm_selection_matrix(0, 3) = params_.cmm_selection_matrix_x;
  cmm_selection_matrix(1, 4) = params_.cmm_selection_matrix_y;
  cmm_selection_matrix(2, 5) = params_.cmm_selection_matrix_z;

  Eigen::VectorXd desired_qddot(6 + n_joints_);
  desired_qddot << Eigen::VectorXd::Zero(6), desired.qjntddot;
  Eigen::VectorXd a_jnt_total = desired_qddot + params_.Kp_regulation * err_posture + params_.Kd_regulation * err_posture_vel;
  Eigen::VectorXd a_com_total = desired.com.acc + params_.Kp_motion * err_com + params_.Kd_motion * err_com_vel;
  Eigen::VectorXd a_lwheel_total = desired.lwheel.acc + params_.Kp_motion * err_lwheel + params_.Kd_motion * err_lwheel_vel;
  Eigen::VectorXd a_rwheel_total = desired.rwheel.acc + params_.Kp_motion * err_rwheel + params_.Kd_motion * err_rwheel_vel;
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



  auto q_jnt_dot_min = -robot_model_.velocityLimit.tail(n_joints_);
  auto q_jnt_dot_max = robot_model_.velocityLimit.tail(n_joints_);
  auto q_jnt_min = robot_model_.lowerPositionLimit.tail(n_joints_);
  auto q_jnt_max = robot_model_.upperPositionLimit.tail(n_joints_);
  

  // RICONTROLLA C_ACC ROMPE IL CODICE
  Eigen::MatrixXd C_acc = Eigen::MatrixXd::Zero(2 * n_joints_, 6 + n_joints_);
  Eigen::VectorXd d_min_acc(2 * n_joints_);
  Eigen::VectorXd d_max_acc(2 * n_joints_);
  C_acc.rightCols(n_joints_).topRows(n_joints_).diagonal().setConstant(sample_time_);
  C_acc.rightCols(n_joints_).bottomRows(n_joints_).diagonal().setConstant(std::pow(sample_time_, 2.0) / 2.0);
  d_min_acc << q_jnt_dot_min - qdot_current, q_jnt_min - q_current - sample_time_ * qdot_current;
  d_max_acc << q_jnt_dot_max - qdot_current, q_jnt_max - q_current - sample_time_ * qdot_current;

  

  Eigen::MatrixXd M = pinocchio::crba(robot_model_, robot_data_, q);
  // We need to do this since the inertia matrix in Pinocchio is only upper triangular
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  M.diagonal().tail(n_joints_) += M_armature_;


  // Computing Coriolis, centrifugal and gravitational effects
  const auto& c = pinocchio::rnea(robot_model_, robot_data_, q, qdot, Eigen::VectorXd::Zero(6 + n_joints_));


  Eigen::MatrixXd Jlu = J_left_wheel_.block(0,0,6,6);
  Eigen::MatrixXd Jla = J_left_wheel_.block(0,6,6,n_joints_);
  Eigen::MatrixXd Jru = J_right_wheel_.block(0,0,6,6);
  Eigen::MatrixXd Jra = J_right_wheel_.block(0,6,6,n_joints_);

  Eigen::MatrixXd Mu = M.block(0,0,6,6+n_joints_);                // fb + n_joints
  Eigen::MatrixXd Ma = M.block(6,0,n_joints_,6+n_joints_);        // n_joints

  Eigen::VectorXd cu = c.block(0,0,6,1);
  Eigen::VectorXd ca = c.block(6,0,n_joints_,1);

  // std::vector<Eigen::Vector3d> pcis(4);
  // pcis[0] <<  params_.foot_length / 2.0,  params_.foot_width / 2.0, 0.0;
  // pcis[1] <<  params_.foot_length / 2.0, -params_.foot_width / 2.0, 0.0;
  // pcis[2] << -params_.foot_length / 2.0,  params_.foot_width / 2.0, 0.0;
  // pcis[3] << -params_.foot_length / 2.0, -params_.foot_width / 2.0, 0.0;

  Eigen::Vector3d pcis(0.0, 0.0, 0.0);
  // Eigen::Vector3d pcis_l = desired.lwheel.pos.R * pcis;         // check this!!!!
  Eigen::Vector3d pcis_l = current_lwheel_pos.R * pcis;         // wrong!!!!
  // Eigen::Vector3d pcis_r = desired.rwheel.pos.R * pcis;
  Eigen::Vector3d pcis_r = current_rwheel_pos.R * pcis;
 
  Eigen::MatrixXd T_l(6, 3 * n_contacts_);
  Eigen::MatrixXd T_r(6, 3 * n_contacts_);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  T_l << I3,
         pinocchio::skew(pcis_l);
  T_r << I3,
         pinocchio::skew(pcis_r);


  Eigen::MatrixXd H_force_one = 1e-9 * Eigen::MatrixXd::Identity(3 * n_contacts_, 3 * n_contacts_);
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

  Eigen::MatrixXd A_acc = Eigen::MatrixXd::Zero(12, 6 + n_joints_);
  Eigen::VectorXd b_acc = Eigen::VectorXd::Zero(12);
  // no contact kept in case of need
  Eigen::MatrixXd A_no_contact = Eigen::MatrixXd::Zero(3 * n_contacts_, 2 * 3 * n_contacts_);   
  Eigen::VectorXd b_no_contact = Eigen::VectorXd::Zero(3 * n_contacts_);

  // constraint: a_contact = J_contact * qddot + J_dot * qdot = 0 + Kd*(0 - pdot) //+ Kp*(p_d - p) wherever is the contact point
  A_acc.topRows(6) = J_left_wheel_ ;
  b_acc.topRows(6) = -J_left_wheel_dot_ * qdot - params_.gamma * J_left_wheel_ * qdot;

  A_acc.bottomRows(6) = J_right_wheel_;   // THIS IS WRONG AND MAKE THE SIMULATION FAIL
  b_acc.bottomRows(6) = -J_right_wheel_dot_ * qdot - params_.gamma * J_right_wheel_ * qdot;

  A_acc *= 0.0; // REMOVED for deubg
  b_acc *= 0.0;

  Eigen::MatrixXd A_dyn(6, 6 + n_joints_ + 2 * 3 * n_contacts_);
  A_dyn << Mu, -Jlu.transpose() * T_l, -Jru.transpose() * T_r;

  // Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_acc.rows() + A_no_contact.rows() + A_dyn.rows(), n_wbc_variables_);
  // A.block(0, 0, A_acc.rows(), A_acc.cols()) = A_acc;
  // A.block(A_acc.rows(), A_acc.cols(), A_no_contact.rows(), A_no_contact.cols()) = A_no_contact;
  // A.bottomRows(A_dyn.rows()) = A_dyn;
  // Eigen::VectorXd b(b_acc.rows() + b_no_contact.rows() + b_dyn.rows());
  // b << b_acc, b_no_contact, b_dyn;


  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_acc.rows() + A_dyn.rows(), n_wbc_variables_);
  A.block(0, 0, A_acc.rows(), A_acc.cols()) = A_acc;
  A.bottomRows(A_dyn.rows()) = A_dyn;
  Eigen::VectorXd b(b_acc.rows() + b_dyn.rows());
  b << b_acc, b_dyn;



  C_acc *= 0.0;       // occhio C_acc rompe il codice
  d_min_acc *= 0.0;
  d_max_acc *= 0.0;


  Eigen::MatrixXd C_force_left = Eigen::MatrixXd::Zero(4 * n_contacts_, 3 * n_contacts_);
  for (int i = 0; i < n_contacts_; ++i) {
    C_force_left.block(4 * i, 3 * i, 4, 3) = C_force_block * current_lwheel_pos.R.transpose();
  }
  Eigen::MatrixXd C_force_right = Eigen::MatrixXd::Zero(4 * n_contacts_, 3 * n_contacts_);
  for (int i = 0; i < n_contacts_; ++i) {
    C_force_right.block(4 * i, 3 * i, 4, 3) = C_force_block * current_rwheel_pos.R.transpose();
  }
  Eigen::MatrixXd C(C_acc.rows() + 2 * C_force_left.rows(), n_wbc_variables_);
  C << C_acc, Eigen::MatrixXd::Zero(C_acc.rows(), 2 * 3 * n_contacts_),
      Eigen::MatrixXd::Zero(C_force_left.rows(), 6 + n_joints_), C_force_left, Eigen::MatrixXd::Zero(C_force_left.rows(), 3 * n_contacts_),
      Eigen::MatrixXd::Zero(C_force_right.rows(), 6 + n_joints_), Eigen::MatrixXd::Zero(C_force_right.rows(), 3 * n_contacts_), C_force_right;
  Eigen::VectorXd d_min(d_min_acc.rows() + 2 * d_min_force_one.rows());
  Eigen::VectorXd d_max(d_max_acc.rows() + 2 * d_max_force_one.rows());
  d_min << d_min_acc, d_min_force_one, d_min_force_one;
  d_max << d_max_acc, d_max_force_one, d_max_force_one;





  // std::cout << "err_posture" << err_posture << std::endl;

  


  wbc_solver_ptr_->solve(H, f, A, b, C, d_min, d_max);
  Eigen::VectorXd solution = wbc_solver_ptr_->get_solution();
  Eigen::VectorXd q_ddot = solution.head(6 + n_joints_);
  Eigen::VectorXd flr = solution.tail(2 * 3 * n_contacts_);
  Eigen::VectorXd fl = flr.head(3 * n_contacts_);
  Eigen::VectorXd fr = flr.tail(3 * n_contacts_);
  Eigen::VectorXd tau = Ma * q_ddot + ca - Jla.transpose() * T_l * fl - Jra.transpose() * T_r * fr;


  if (count<3){
    std::cout << "q_ddot" << q_ddot << std::endl;
  }
  count++;

  std::cout << "computed tau" << tau << std::endl;


  // Fine misurazione del tempo
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // Stampa del tempo di esecuzione
  // std::cout << "Tempo di esecuzione del controllore Whole Body: " << duration << " microsecondi" << std::endl;


  JointCommand joint_command;
  for(pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex) robot_model_.njoints; ++joint_id) {
    const auto& joint_name = robot_model_.names[joint_id];
    joint_command[joint_name] = tau[joint_id - 2];
    //joint_command[joint_name] = 0.0;
  }
  
  return joint_command;
}

}