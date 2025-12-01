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

  struct Zmp {
    Eigen::Vector3d pos; 
    Eigen::Vector3d vel; 
    Eigen::Vector3d acc;
  };

  Com com;
  Zmp zmp;
};


class MPC {
  static constexpr int SOLVER_MAX_ITER = 1;         // 10
  static constexpr int NX = 10;                     // state size
  static constexpr int NU = 5;                      // input size
  static constexpr int NE = 3;                      // eq.
  static constexpr int NY = 3;                      // terminal constraint size
  static constexpr int NY_zmp = 2;
  static constexpr int NC = 2; //2*(NY-1); //2 * NY;                 // inequality constraint size
  static constexpr int NH = 400;          //400     // horizon length
  typedef Eigen::Matrix<double, NX, 1> VectorX;
  typedef Eigen::Matrix<double, NU, 1> VectorU;

  public:

  MPC(Eigen::Vector<double, NX> x0){

    C_zmp << 0,0,0, 0,0,0, 1,0, 0,0,
             0,0,0, 0,0,0, 0,1, 0,0;


    Cv_zmp << 0,0,0, 0,0,0, 0,0, 1,0,
              0,0,0, 0,0,0, 0,0, 0,1;

    C_com << 1,0,0, 0,0,0, 0,0, 0,0,
             0,1,0, 0,0,0, 0,0, 0,0,
             0,0,1, 0,0,0, 0,0, 0,0;

    Cv_com << 0,0,0, 1,0,0, 0,0, 0,0,
              0,0,0, 0,1,0, 0,0, 0,0,
              0,0,0, 0,0,1, 0,0, 0,0;

    for (int i = 0; i < NH+1; ++i)
    {
        zmp_ref(0,i) = 0.0;
        zmp_ref(1,i) = 0.0;

        pcom_ref(0,i) = 0.0;
        pcom_ref(1,i) = 0.0;
        pcom_ref(2,i) = 0.4;
    }


    // Dynamics
    Eigen::Matrix<double, 3, 3> I3 = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 2, 2> I2 = Eigen::Matrix<double, 2, 2>::Identity();
    A.block(0,3, 3, 3) = I3; 
    A.block(6,8, 2, 2) = I2; 

    B.block(3,2, 3,3) = 1/m * I3;
    B.block(8,0, 2,2) = I2;

    c << 0,0,0, 0,0,-grav, 0,0, 0,0;

        // u0 << 1.0,       // works only with this
        //       1.0,
        //       1.0,
        //       1.0,
        //       500.389;

    initSolver(x0);

  };

  
  SolutionMPC get_solution() const {
    return {
      {pos_com_, vel_com_, acc_com_},   // COM
      {pos_zmp_, vel_zmp_, acc_zmp_}    // ZMP
    };
  }
  

  void initSolver(Eigen::Vector<double, NX> x0);


  void set_reference_trajectory(Eigen::Matrix<double, NY, NH+1>& traj_ref){
    // zmp_ref = traj_ref;
  }


  void solve(Eigen::Vector<double, NX> x0);

  bool record_logs = false;
  double t_msec = 0.0;

private:
  std::unique_ptr<DdpSolver<NX, NU, NE, NY_zmp, NC, NH>> solver_ptr_;

  Eigen::Vector3d pos_com_, vel_com_, acc_com_, pos_zmp_, vel_zmp_, acc_zmp_;

  // parameters
  double grav = 9.81;                  // gravity
  double Δ = 0.002;                    // time step
  double m = 44.0763;
  double z_c = 0.0;

  // cost function weights
  double w_z = 20.0;                   // ZMP tracking weight
  double w_cd = 1.0;                   // COM vel tracking weight
  double w_zd = 0.00001;               // input weight
  double w_acc = 1e-7;  
  
  double w_fxy = 1e-8;    
  double w_fz = 1e-8;
  

  double w_h = 160.0;   // 5200.0
  double w_vh = 1.0;  // 60.0

  


  // output matrices
  Eigen::Matrix<double, NY, NX> C_com, Cv_com;
  Eigen::Matrix<double, NY_zmp, NX> C_zmp, Cv_zmp;

  // zmp ref trajectory
  Eigen::Matrix<double, NY, NH+1> pcom_ref = Eigen::Matrix<double, NY, NH + 1>::Zero();
  Eigen::Matrix<double, NY_zmp, NH+1> zmp_ref = Eigen::Matrix<double, NY_zmp, NH + 1>::Zero();

  // dynamics
  Eigen::Matrix<double, NX, NX> A = Eigen::Matrix<double, NX, NX>::Zero();
  Eigen::Matrix<double, NX, NU> B = Eigen::Matrix<double, NX, NU>::Zero();
  Eigen::Matrix<double, NX, 1> c = Eigen::Matrix<double, NX, 1>::Zero();

  // previous guess
  Eigen::Vector<double, NU> u0 = Eigen::Vector<double, NU>::Zero();

}; 

} // end namespace labrob
