#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <SE3.hpp>

namespace labrob {
struct ee3 {
  Eigen::Vector3d pos; Eigen::Vector3d vel; Eigen::Vector3d acc;
};

struct ee_rot {
  Eigen::Matrix3d pos; Eigen::Vector3d vel; Eigen::Vector3d acc;
};

struct ee6 {
  labrob::SE3 pos; Eigen::Vector<double, 6> vel; Eigen::Vector<double, 6> acc;
};

struct DesiredConfiguration {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  // Velocities of the base link expressed in the reference frame of the base.
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  Eigen::VectorXd qjnt, qjntdot, qjntddot;
  
  labrob::ee3 com;
  labrob::ee6 lwheel, rwheel; // wheel frame position + orientation
  labrob::ee_rot base_link;
  bool in_contact;

};

} // end namespace labrob