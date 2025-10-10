#pragma once

// Pinocchio
#include <pinocchio/multibody/model.hpp>

#include <RobotState.hpp>
#include <mujoco/mujoco.h>

namespace labrob {

Eigen::Matrix<double, 6, 1>
err_frameplacement(const pinocchio::SE3& Ta, const pinocchio::SE3& Tb);

Eigen::Vector3d
err_translation(const Eigen::Vector3d& pa, const Eigen::Vector3d& pb);

Eigen::Vector3d
err_rotation(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb);

Eigen::VectorXd
robot_state_to_pinocchio_joint_configuration(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
);

Eigen::VectorXd
robot_state_to_pinocchio_joint_velocity(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
);

RobotState robot_state_from_mujoco(mjModel* m, mjData* d);

Eigen::Vector3d get_rCP(const Eigen::Vector3d& wheel_center, const Eigen::MatrixXd& wheel_R, const double& wheel_radius);
Eigen::Matrix3d compute_contact_frame(const Eigen::Vector3d& wheel_center, const Eigen::MatrixXd& wheel_R, const double& wheel_radius);

} // end namespace labrob