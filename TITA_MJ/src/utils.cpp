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


RobotState robot_state_from_mujoco(mjModel* m, mjData* d) {
labrob::RobotState robot_state;

robot_state.position = Eigen::Vector3d(
  d->qpos[0], d->qpos[1], d->qpos[2]
);

robot_state.orientation = Eigen::Quaterniond(
    d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]
);

robot_state.linear_velocity = robot_state.orientation.toRotationMatrix().transpose() *
    Eigen::Vector3d(
        d->qvel[0], d->qvel[1], d->qvel[2]
    );

robot_state.angular_velocity = Eigen::Vector3d(
  d->qvel[3], d->qvel[4], d->qvel[5]
);

for (int i = 1; i < m->njnt; ++i) {
  const char* name = mj_id2name(m, mjOBJ_JOINT, i);
  robot_state.joint_state[name].pos = d->qpos[m->jnt_qposadr[i]];
  robot_state.joint_state[name].vel = d->qvel[m->jnt_dofadr[i]];
}

static double force[6];
static double result[3];
Eigen::Vector3d sum = Eigen::Vector3d::Zero();
robot_state.contact_points.resize(d->ncon);
robot_state.contact_forces.resize(d->ncon);
for (int i = 0; i < d->ncon; ++i) {
  mj_contactForce(m, d, i, force);
  //mju_rotVecMatT(result, force, d->contact[i].frame);
  mju_mulMatVec(result, d->contact[i].frame, force, 3, 3);
  for (int row = 0; row < 3; ++row) {
      result[row] = 0;
      for (int col = 0; col < 3; ++col) {
          result[row] += d->contact[i].frame[3 * col + row] * force[col];
      }
  }
  sum += Eigen::Vector3d(result);
  for (int j = 0; j < 3; ++j) {
    robot_state.contact_points[i](j) = d->contact[i].pos[j];
    robot_state.contact_forces[i](j) = result[j];
  }
}

robot_state.total_force = sum;

return robot_state;
}

Eigen::Vector3d get_rCP(const Eigen::Vector3d& wheel_center, const Eigen::MatrixXd& wheel_R, const double& wheel_radius){
  Eigen::Vector3d z_0 = Eigen::Vector3d(0,0,1);
  Eigen::Vector3d n = wheel_R * z_0;
  Eigen::Vector3d t = (Eigen::Matrix3d::Identity() - n*n.transpose()) * z_0;
  t = t/(t.norm()); // normalize
  Eigen::Vector3d contact_point = wheel_center - t * wheel_radius;

  Eigen::Vector3d rCP = contact_point-wheel_center;
  return rCP;
}
Eigen::Matrix3d compute_contact_frame(const Eigen::Vector3d& wheel_center, const Eigen::MatrixXd& wheel_R, const double& wheel_radius){
  Eigen::Vector3d z_0 = Eigen::Vector3d(0,0,1);
  Eigen::Vector3d s = wheel_R * z_0;
  Eigen::Vector3d n = (Eigen::Matrix3d::Identity() - s*s.transpose()) * z_0;
  n = n/(n.norm()); // normalize
  Eigen::Vector3d t = s.cross(n);
  t = t/(t.norm()); // normalize
  Eigen::Matrix3d R;
  R.col(0) = t;  
  R.col(1) = s;   
  R.col(2) = n;
  return R;
}

} // end namespace labrob