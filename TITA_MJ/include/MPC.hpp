#pragma once

#include <DFIPActionModel.hpp>
#include <walkingPlanner.hpp>
#include <utils.hpp>

#include <iostream>
#include <fstream>

namespace labrob {

struct SolutionMPC { 
  struct Com { Eigen::Vector3d pos;  Eigen::Vector3d vel;  Eigen::Vector3d acc; };
  struct Pl {Eigen::Vector3d pos;  Eigen::Vector3d vel;  Eigen::Vector3d acc; };
  struct Pr {Eigen::Vector3d pos;  Eigen::Vector3d vel;  Eigen::Vector3d acc; };

  Com com;
  Pl pl;
  Pr pr;
  double theta;
  double omega;
  double alpha;
};


class MPC {
  static constexpr int SOLVER_MAX_ITER = 1;    
  static constexpr int N_IN = 18; 
  static constexpr int NX = 13; 
  static constexpr int NU = 9;          
  static constexpr int NH = 50;

  public:
  MPC(){};

  void set_planner(const labrob::walkingPlanner& planner, const double& dt) {
    walkingPlanner_ptr_ = &planner;
    dt_ = dt;
  }

  Eigen::Vector<double, NX> get_DFIP_state(Eigen::Vector<double, N_IN> x0, bool set_d);

  void init_solver(Eigen::Vector<double, N_IN> x0);
  
  SolutionMPC get_solution() const {
    return {
      {pos_com_, vel_com_, acc_com_},   // COM
      {pos_pl_, vel_pl_, acc_pl_},       // pl
      {pos_pr_, vel_pr_, acc_pr_},       // pr
      theta_,
      omega_,
      alpha_
    };
  }

  void update_actionModel();

  void solve(Eigen::Vector<double, N_IN> x0);

  bool record_logs = false;
  double t_msec = 0.0;
  

private:
  const labrob::walkingPlanner* walkingPlanner_ptr_ = nullptr;

  Eigen::Vector3d pos_com_, vel_com_, acc_com_, pos_pl_, vel_pl_, acc_pl_, pos_pr_, vel_pr_, acc_pr_;
  double theta_, omega_, alpha_;

  // VHIP parameters
  double grav = 9.81;                   // gravity
  double Δ    = 0.01;                   // prediction step
  double dt_  = 0.002;                  // control timestep
  double m    = 27.68978;
  double d    = 0.1;                    // unicycle offset

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
