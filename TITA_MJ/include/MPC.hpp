#include "DdpSolver.hpp"
#include <iostream>
#include <fstream>

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
  static constexpr int NX = 3;                      // state size
  static constexpr int NU = 1;                      // input size
  static constexpr int NY = 1;                      // terminal constraint size
  static constexpr int NC = 2 * NY;                 // inequality constraint size
  static constexpr int NH = 400;          //400     // horizon length
  typedef Eigen::Matrix<double, NX, 1> VectorX;
  typedef Eigen::Matrix<double, NU, 1> VectorU;

  public:

  MPC(){
    // C_zmp << 0, 0, 1, 0, 0, 0, 0, 0, 0,
    //          0, 0, 0, 0, 0, 1, 0, 0, 0,
    //          0, 0, 0, 0, 0, 0, 0, 0, 1;
    // C_com << 1, 0, 0, 0, 0, 0, 0, 0, 0,
    //          0, 0, 0, 1, 0, 0, 0, 0, 0,
    //          0, 0, 0, 0, 0, 0, 1, 0, 0;

    // // compute reference trajectory
    // // ogni volta la ricalcola, trova altro modo per assegnare la ref traj
    // double step_x = 0.0; //0.1;
    // double step_y = 0.0; //0.1;
    // for (int i = 0; i < NH+1; ++i)
    // {
    //     int step_index = floor(i / 100);
    //     int left_right = step_index % 2 == 0 ? 1 : -1;
    //     zmp_ref(0,i) = step_x * step_index;
    //     zmp_ref(1,i) = step_y * left_right;
    //     zmp_ref(2,i) = 0.095;
    // }

    C_zmp << 0, 0, 1;
    C_com << 1, 0, 0;
    Cv_com << 0, 1, 0;
  };

  
  SolutionMPC get_solution() const {
    return {
      {pos_com_, vel_com_, acc_com_},   // COM
      {pos_zmp_, vel_zmp_, acc_zmp_}    // ZMP
    };
  }
  
  void set_pendulum_height(double h_des){
    h = h_des;
    η = sqrt(grav/h);

    // // dynamics
    // Eigen::Matrix<double, 3, 3> A_LIP;
    // A_LIP << 0, 1, 0, η*η, 0, -η*η, 0, 0, 0;
    // Eigen::Matrix<double, 3, 1> B_LIP;
    // B_LIP << 0, 0, 1;
    // //   static Eigen::Matrix<double, MPC::NX, 1> c = Eigen::Matrix<double, MPC::NX, 1>::Zero();
    // //   c << 0, 0, 0, 0, 0, 0, 0, -grav, 0;

    // for (int i = 0; i < 3; ++i)
    //   {
    //     A.block<3,3>(3*i,3*i) = A_LIP;
    //     B.block<3,1>(3*i,i)   = B_LIP;
    //     if (i == 2){
    //       A.block<3,3>(3*i,3*i) << 0,1,0,  0,0,0,  0,0,0; // not controlled z
    //     }
    //   }

    // dynamics
    A << 0, 1, 0, η*η, 0, -η*η, 0, 0, 0;
    B << 0, 0, 1;
  }

  void set_reference_trajectory(Eigen::Matrix<double, NY, NH+1>& traj_ref){
    zmp_ref = traj_ref;
  }


  void solve(Eigen::Vector<double, NX> x0, Eigen::Vector3d curr_zmp_vel);

  bool record_logs = false;

private:

  Eigen::Vector3d pos_com_, vel_com_, acc_com_, pos_zmp_, vel_zmp_, acc_zmp_;

  // LIP parameters
  double h;                           // CoM height
  double grav = 9.81;                 // gravity
  double η;                           // pendulum natural pulse
  double Δ = 0.01;                    // time step

  // cost function weights
  double w_z = 20.0;          //20.0     // ZMP tracking weight
  double w_cd = 20.0;          //20.0     // COM vel tracking weight
  double w_zd = 0.001;        //0.001          // input weight

  // output matrices
  Eigen::Matrix<double, NY, NX> C_zmp, C_com, Cv_com;

  // zmp ref trajectory
  Eigen::Matrix<double, NY, NH+1> zmp_ref = Eigen::Matrix<double, NY, NH + 1>::Zero();

  // dynamics
  Eigen::Matrix<double, NX, NX> A = Eigen::Matrix<double, NX, NX>::Zero();
  Eigen::Matrix<double, NX, NU> B = Eigen::Matrix<double, NX, NU>::Zero();

}; 

} // end namespace labrob
