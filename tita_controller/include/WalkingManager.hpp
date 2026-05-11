#pragma once

#include <WholeBodyController.hpp>
#include <DesiredConfiguration.hpp>
#include <MPC.hpp>
// #include <LQR.hpp>
// #include <walkingPlanner.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>
#include <Logger.hpp>


namespace labrob {

class WalkingManager {
 public:

  bool init(const labrob::RobotState& initial_robot_state,
    std::map<std::string, double> &armatures,
    const pinocchio::Model& robot_model);

  void update(
      const labrob::RobotState& robot_state,
      labrob::JointCommand& joint_torque, 
      labrob::JointCommand& joint_acceleration,
      double t_msec_
  );

  void save_data(const std::string& log_directory);
  
  labrob::DesiredConfiguration des_configuration_;
  

 protected:
  const pinocchio::Model* robot_model_;
  pinocchio::Data robot_data_;
  pinocchio::FrameIndex right_leg4_idx_;
  pinocchio::FrameIndex left_leg4_idx_;
  pinocchio::FrameIndex base_idx_;

  double wheel_radius_;

  double controller_timestep_msec_;

  std::shared_ptr<labrob::WholeBodyController> whole_body_controller_ptr_;

private:

  double controller_frequency_;
  int cycle_counter = 0;


  labrob::walkingPlanner walkingPlanner_;
  labrob::MPC mpc_;
  // labrob::LQR lqr_;


  // Logger:
  labrob::Logger logger_;

}; 

} // end namespace labrob
