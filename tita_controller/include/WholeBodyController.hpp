#pragma once

// Pinocchio
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>    
// #include <pinocchio/algorithm/kinematics.hpp>              //|--> not necessary
#include <pinocchio/algorithm/model.hpp>                   
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>


#include <JointCommand.hpp>
// #include <RobotState.hpp>        //included in utils.hpp
#include <DesiredConfiguration.hpp>
#include <utils.hpp>

#include <labrob_qpsolvers/qpsolvers.hpp>

namespace labrob {

struct WholeBodyControllerParams {
  double Kp_motion;
  double Kd_motion;
  double Kp_regulation;
  double Kd_regulation;

  double Kp_wheel;
  double Kd_wheel;

  double weight_q_ddot;
  double weight_com;
  double weight_lwheel;
  double weight_rwheel;
  double weight_base;
  double weight_regulation;
  double weight_angular_momentum;

  double cmm_selection_matrix_x;
  double cmm_selection_matrix_y;
  double cmm_selection_matrix_z;

  double mu;

  double weight_tau_reg;

  static WholeBodyControllerParams getDefaultParams();
  static WholeBodyControllerParams getRobustParams();
  static WholeBodyControllerParams getJumpParams();
};

class WholeBodyController {
 public:
  WholeBodyController(const WholeBodyControllerParams& params,
                      const pinocchio::Model& robot_model,
                      double sample_time,
                      std::map<std::string, double>& armature);

  void
  compute_inverse_dynamics(
      const labrob::RobotState& robot_state,
      const labrob::DesiredConfiguration& desired,
      labrob::JointCommand& joint_torque, 
      labrob::JointCommand& joint_acceleration
  );

  double wheel_radius_;
  double wheel_width_;
  double wheel_length_;

  WholeBodyControllerParams params_;
  
 private:
  const pinocchio::Model* robot_model_;
  pinocchio::Data robot_data_;

  pinocchio::FrameIndex right_leg4_idx_;
  pinocchio::FrameIndex left_leg4_idx_;
  pinocchio::FrameIndex base_link_idx_;

  Eigen::MatrixXd J_right_wheel_;
  Eigen::MatrixXd J_left_wheel_;
  Eigen::MatrixXd J_base_link_;

  Eigen::MatrixXd J_right_wheel_dot_;
  Eigen::MatrixXd J_left_wheel_dot_;
  Eigen::MatrixXd J_base_link_dot_;

  double sample_time_;

  Eigen::VectorXd M_armature_;

  int n_joints_;
  int n_contacts_;
  int n_wbc_variables_;
  int n_wbc_equalities_;
  int n_wbc_inequalities_;

  std::unique_ptr<qpsolvers::QPSolverEigenWrapper<double>> wbc_solver_ptr_;

};
}