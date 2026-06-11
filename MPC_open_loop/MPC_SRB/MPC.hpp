#pragma once

#include <DFIPActionModel.hpp>
#include <walkingPlanner.hpp>
#include <utils.hpp>

#include <iostream>
#include <fstream>
#include <cmath>

namespace labrob {

struct SolutionMPC { 
  struct Com { Eigen::Vector3d pos;  Eigen::Vector3d vel;  Eigen::Vector3d acc; };
  struct RPY { Eigen::Vector3d pos;  Eigen::Vector3d vel; Eigen::Vector3d acc; };
  struct Pr {Eigen::Vector3d pos;  Eigen::Vector3d vel;  Eigen::Vector3d acc; };
  struct Pl {Eigen::Vector3d pos;  Eigen::Vector3d vel;  Eigen::Vector3d acc; };

  Com com;
  RPY rpy;
  Pr pr;
  Pl pl;
};


class MPC {
  static constexpr int SOLVER_MAX_ITER = 5;    
  static constexpr int NX = 22; 
  static constexpr int NU = 10;          
  static constexpr int NH = 10; // 50

  public:
  MPC(){};

  void set_planner(const labrob::walkingPlanner& planner, const double& dt) {
    walkingPlanner_ptr_ = &planner;
    dt_ = dt;
  }

  SolutionMPC get_solution() const {
    return {
      {pos_com_, vel_com_, acc_com_},   // COM
      {pos_rpy_, vel_rpy_, acc_rpy_},       // RPY
      {pos_pr_, vel_pr_, acc_pr_},       // pr
      {pos_pl_, vel_pl_, acc_pl_},       // pl
    };
  }
  
  void update_actionModel();
  
  void init_solver(Eigen::Vector<double, NX> x0);

  void solve(Eigen::Vector<double, NX> x0);

  double get_nominal_dt() {return dt_;}

  void set_current_t_msec(const double& t_msec) {t_msec_ = t_msec;}

  // used for logging... find a better way
  Eigen::MatrixXd X;
  Eigen::MatrixXd U;
  

private:
  const labrob::walkingPlanner* walkingPlanner_ptr_ = nullptr;

  Eigen::Vector3d pos_com_, vel_com_, acc_com_, 
                  pos_pl_, vel_pl_, acc_pl_, 
                  pos_pr_, vel_pr_, acc_pr_, 
                  pos_rpy_, vel_rpy_, acc_rpy_;

  // VHIP parameters
  double grav = 9.81;                   // gravity
  double Δ    = 0.01;                   // prediction step
  double dt_  = 0.002;                  // control timestep
  double m    = 27.68978;
  double d    = 0.1;
  Eigen::Matrix3d I;
  Eigen::Matrix3d I_inv_;

  // external time; 
  double t_msec_ = 0.0;

  // TODO: handle the state in the manifold
  double theta_prev_ = 0.0;             // needed to unwrap theta \in SO(2) - > R

  // initial guess trajectory
  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> us;


  // FDDP solver
  std::shared_ptr<SolverFDDP> solver;

  std::vector<std::shared_ptr<DFIPActionModel>> di_models_;
  std::shared_ptr<DFIPActionModel> terminalModel_;
  std::shared_ptr<ShootingProblem> problemPtr_;

}; 

} // end namespace labrob
