#pragma once

#include <WholeBodyController.hpp>
#include <DesiredConfiguration.hpp>
#include <MPC.hpp>
#include <LQR.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>


namespace labrob {

class WalkingManager {
 public:

  bool init(const labrob::RobotState& initial_robot_state, std::map<std::string, double> &armatures);

  void update(
      const labrob::RobotState& robot_state,
      labrob::JointCommand& joint_command
  );

  labrob::DesiredConfiguration des_configuration_;


 protected:
  pinocchio::Model robot_model_;
  pinocchio::Data robot_data_;
  pinocchio::FrameIndex right_leg4_idx_;
  pinocchio::FrameIndex left_leg4_idx_;

  double controller_timestep_msec_;

  std::shared_ptr<labrob::WholeBodyController> whole_body_controller_ptr_;

private:

  double controller_frequency_;
  double t_msec_ = 0;


  labrob::MPC mpc_;
  Eigen::Matrix<double, 2, 200+1> pc_ref;
  Eigen::Matrix<double, 3, 200+1> pcom_ref;

  // Log files:
  // std::ofstream mpc_timings_log_file_;
  std::ofstream state_log_file_;

}; 

} // end namespace labrob
