#pragma once

#include "DdpSolver.hpp"
#include <iostream>
#include <fstream>

#include <pinocchio/algorithm/centroidal.hpp>     // to use pinocchio::skew

namespace labrob {

struct SolutionMPC { 

  struct Com {
    Eigen::Vector3d pos; 
    Eigen::Vector3d vel; 
    Eigen::Vector3d acc;
  };

  struct Pc {
    Eigen::Vector3d pos; 
    Eigen::Vector3d vel; 
    Eigen::Vector3d acc;
  };

  Com com;
  Pc pc;
};

struct QP_Z { 
  static constexpr int NX = 2;                      // state size
  static constexpr int NU = 1;                      // input size
  static constexpr int NY = 1;             

  typedef Eigen::Matrix<double, NX, 1> VectorX;
  typedef Eigen::Matrix<double, NU, 1> VectorU;


  Eigen::Matrix<double, NX, NX> A = Eigen::Matrix<double, NX, NX>::Zero();
  Eigen::Matrix<double, NX, NU> B = Eigen::Matrix<double, NX, NU>::Zero();
  Eigen::Matrix<double, NX, 1> c = Eigen::Matrix<double, NX, 1>::Zero();

  // output matrices
  Eigen::Matrix<double, NY, NX> C_com, Cv_com;

  Eigen::Matrix<double, NU, 1> u_prev = Eigen::Vector<double, NU>::Zero();

  QP_Z(double m, double grav)
  {

      C_com << 1,0;
      Cv_com << 0,1;

      //dynmaics
      A(0,1) = 1.0;
      B(1,0) = 1.0/m;
      c << 0,-grav;

      u_prev << m*grav;
  }
};

struct QP_XY {

  static constexpr int NX = 8;
  static constexpr int NU = 2;
  static constexpr int NY = 2;

  typedef Eigen::Matrix<double, NX, 1> VectorX;
  typedef Eigen::Matrix<double, NU, 1> VectorU;

  Eigen::Matrix<double, NX, NX> A = Eigen::Matrix<double, NX, NX>::Zero();
  Eigen::Matrix<double, NX, NU> B = Eigen::Matrix<double, NX, NU>::Zero();

  // output matrices
  Eigen::Matrix<double, NY, NX> C_com, Cv_com, C_pc, Cv_pc;

  Eigen::Matrix<double, NU, 1> u_prev = Eigen::Vector<double, NU>::Zero();

  QP_XY()
  {
      C_pc << 0,0, 0,0, 1,0, 0,0,
               0,0, 0,0, 0,1, 0,0;

      Cv_pc << 0,0, 0,0, 0,0, 1,0,
                0,0, 0,0, 0,0, 0,1;

      C_com << 1,0, 0,0, 0,0, 0,0,
               0,1, 0,0, 0,0, 0,0;

      Cv_com << 0,0, 1,0, 0,0, 0,0,
                0,0, 0,1, 0,0, 0,0;
      
      u_prev << 0,0;
  }
  };


class MPC {
  static constexpr int SOLVER_MAX_ITER = 1;    
  static constexpr int N_STATE = 10;          
  static constexpr int N_OUTPUT = 3;         
  static constexpr int NH = 200;

  public:

  MPC(): Qp_z(m, grav){};

  
  SolutionMPC get_solution() const {
    return {
      {pos_com_, vel_com_, acc_com_},   // COM
      {pos_pc_, vel_pc_, acc_pc_}       // ZMP
    };
  }

  void set_reference_trajectory(Eigen::Matrix<double, N_OUTPUT-1, NH + 1>& pc_traj_ref, Eigen::Matrix<double, N_OUTPUT, NH + 1>& pcom_traj_ref){
    pcom_ref = pcom_traj_ref;
    pc_ref = pc_traj_ref;
  }

  void solve(Eigen::Vector<double, N_STATE> x0);

  bool record_logs = false;
  double t_msec = 0.0;

private:

  Eigen::Vector3d pos_com_, vel_com_, acc_com_, pos_pc_, vel_pc_, acc_pc_;

  // VHIP parameters
  double grav = 9.81;                 // gravity
  double Δ = 0.002;                   // time step
  double m = 44.0763;
  double z_c = 0.0;                   // contact point z

  // cost function weights
  double w_z = 20.0;                  // ZMP tracking weight
  double w_zd = 0.0001;               // input weight
  
  double w_acc = 1e-9; 
  double w_fc = 1e-8;  
  
  double w_h = 160.0; 
  double w_vh = 0.01;

  double w_c = 0.0; 
  double w_cd = 1.0;                  // COM vel tracking weight


  // zmp ref trajectory
  Eigen::Matrix<double, N_OUTPUT, NH+1> pcom_ref = Eigen::Matrix<double, N_OUTPUT, NH + 1>::Zero();
  Eigen::Matrix<double, N_OUTPUT-1, NH+1> pc_ref = Eigen::Matrix<double, N_OUTPUT-1, NH + 1>::Zero();

  QP_Z Qp_z;
  QP_XY Qp_xy;
}; 

} // end namespace labrob
