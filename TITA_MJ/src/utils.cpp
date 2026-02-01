#include <utils.hpp>



namespace labrob {

Eigen::Matrix<double, 6, 1>
err_frameplacement(const pinocchio::SE3& Ta, const pinocchio::SE3& Tb) {
  // TODO: how do you use pinocchio::log6?
  Eigen::Matrix<double, 6, 1> err;
  err << err_translation(Ta.translation(), Tb.translation()),
      err_rotation(Ta.rotation(), Tb.rotation());
  return err;
}

Eigen::Vector3d
err_translation(const Eigen::Vector3d& pa, const Eigen::Vector3d& pb) {
  return pa - pb;
}

Eigen::Vector3d
err_rotation(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb) {
  // TODO: how do you use pinocchio::log3?
  Eigen::Matrix3d Rdiff = Rb.transpose() * Ra;
  auto aa = Eigen::AngleAxisd(Rdiff);
  return aa.angle() * Ra * aa.axis();
}

Eigen::VectorXd
robot_state_to_pinocchio_joint_configuration(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
) {
  // labrob::RobotState representation to Pinocchio representation:
  // TODO: RobotState also has information about the velocity of the floating base.
  // TODO: is there a less error-prone way to convert representation?
  Eigen::VectorXd q(robot_model.nq);
  q.head<3>() = robot_state.position;
  q.segment<4>(3) = robot_state.orientation.coeffs();
  // NOTE: start from joint id (2) to skip frames "universe" and "root_joint".
  for(pinocchio::JointIndex joint_id = 2;
      joint_id < (pinocchio::JointIndex) robot_model.njoints;
      ++joint_id) {
    const auto& joint_name = robot_model.names[joint_id];
    q[joint_id + 5] = robot_state.joint_state[joint_name].pos;
  }

  return q;
}

Eigen::VectorXd
robot_state_to_pinocchio_joint_velocity(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
) {
  Eigen::VectorXd qdot(robot_model.nv);
  qdot.head<3>() = robot_state.linear_velocity;
  qdot.segment<3>(3) = robot_state.angular_velocity;
  // NOTE: start from joint id (2) to skip frames "universe" and "root_joint".
  for(pinocchio::JointIndex joint_id = 2;
      joint_id < (pinocchio::JointIndex) robot_model.njoints;
      ++joint_id) {
    const auto& joint_name = robot_model.names[joint_id];
    qdot[joint_id + 4] = robot_state.joint_state[joint_name].vel;
  }
  
  return qdot;
}


double wrapToPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

double unwrapNear(double theta_wrapped, double theta_prev) {
  return theta_prev + wrapToPi(theta_wrapped - theta_prev);
}

Eigen::Vector3d get_rCP(const Eigen::MatrixXd& wheel_R, const double& wheel_radius){
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Vector3d z_0 = Eigen::Vector3d(0,0,1);
  Eigen::Vector3d n = wheel_R * z_0;
  Eigen::Vector3d a = (I - n*n.transpose()) * z_0;
  Eigen::Vector3d s = a/a.norm(); // normalize
  Eigen::Vector3d rCP = - s * wheel_radius;
  return rCP;
}


Eigen::Matrix3d compute_virtual_frame(const Eigen::MatrixXd& wheel_R){
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Vector3d z_0 = Eigen::Vector3d(0,0,1);
  Eigen::Vector3d n = wheel_R * z_0;
  Eigen::Vector3d a = (I - n*n.transpose()) * z_0;
  Eigen::Vector3d s = a / a.norm(); // normalize
  Eigen::Vector3d t = n.cross(s);
  t = t/t.norm(); // normalize
  Eigen::Matrix3d R;
  R.col(0) = t;  
  R.col(1) = n;   
  R.col(2) = s;
  return R;
}

Eigen::Matrix3d compute_contact_frame(const Eigen::MatrixXd& wheel_R){
  Eigen::Vector3d z_0 = Eigen::Vector3d(0,0,1);
  Eigen::Vector3d n = wheel_R * z_0;
  Eigen::Vector3d a = n.cross(z_0);
  Eigen::Vector3d t = a / a.norm(); // normalize
  Eigen::Matrix3d R;
  R.col(0) = t;  
  R.col(1) = z_0.cross(t);   
  R.col(2) = z_0;
  return R;
}


} // end namespace labrob