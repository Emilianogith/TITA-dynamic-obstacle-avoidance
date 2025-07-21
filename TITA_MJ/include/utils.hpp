#ifndef LABROB_UTILS_HPP_
#define LABROB_UTILS_HPP_

// STL
#include <cmath>
#include <deque>

// Eigen
#include <Eigen/Core>

// Pinocchio
#include <pinocchio/multibody/model.hpp>


#include <RobotState.hpp>

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


} // end namespace labrob

#endif // LABROB_UTILS_HPP_