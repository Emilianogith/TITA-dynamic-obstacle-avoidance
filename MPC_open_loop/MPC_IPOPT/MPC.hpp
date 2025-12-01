#pragma once

#include "DdpSolver.hpp"


#include <casadi/casadi.hpp>

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
  static constexpr int NX = 12;                      // state size
  static constexpr int NU = 6;                       // input size
  static constexpr int NY = 3;                       // terminal constraint size
  static constexpr int NH = 50;                      // horizon length

  typedef Eigen::Matrix<double, NX, 1> VectorX;
  typedef Eigen::Matrix<double, NU, 1> VectorU;

  public:

  MPC(){

    // set reference trajectory
    for (int i = 0; i < NH+1; ++i)
    {
        zmp_ref(0,i) = 0.0;
        zmp_ref(1,i) = 0.0;
        zmp_ref(2,i) = 0.0;

        pcom_ref(0,i) = 0.0;
        pcom_ref(1,i) = 0.0;
        pcom_ref(2,i) = 0.4;
    }



    // Non linear optimization problem
    opti_ptr_ = std::make_shared<casadi::Opti>("nlp");
    auto p_opts = casadi::Dict();
    auto s_opts = casadi::Dict();
    p_opts["expand"] = true;
    p_opts["print_time"] = false;
    s_opts["print_level"] = 0;
    s_opts["sb"] = "yes"; 
    s_opts["print_user_options"] = "no";
    s_opts["print_options_documentation"] = "no";
    s_opts["print_timing_statistics"] = "no";
    opti_ptr_->solver("ipopt", p_opts, s_opts);


    // state definition
    std::vector<casadi::MX> pc(NH+1);
    std::vector<casadi::MX> vc(NH+1);
    std::vector<casadi::MX> pcom(NH+1);
    std::vector<casadi::MX> vcom(NH+1);
    std::vector<casadi::MX> fc(NH);
    std::vector<casadi::MX> ac(NH);
    
    casadi::MX cost = 0;

    for (int k = 0; k <= NH; ++k) {
        pcom[k] = opti_ptr_->variable(3,1);
        vcom[k] = opti_ptr_->variable(3,1);
        pc[k]   = opti_ptr_->variable(3,1);
        vc[k]   = opti_ptr_->variable(3,1);
          
        state_vars_.push_back(pcom[k]);
        state_vars_.push_back(vcom[k]);
        state_vars_.push_back(pc[k]);
        state_vars_.push_back(vc[k]);
        if (k < NH){
          ac[k]   = opti_ptr_->variable(3,1);
          fc[k]   = opti_ptr_->variable(3,1);
          input_vars_.push_back(ac[k]);
          input_vars_.push_back(fc[k]);
        }
    }

    for (int k = 0; k < NH; ++k) {

        // dynamics
        opti_ptr_->subject_to(pcom[k+1] == pcom[k] + Δ * vcom[k]);
        opti_ptr_->subject_to(vcom[k+1] == vcom[k] + Δ * (fc[k] / m + g));
        opti_ptr_->subject_to(pc[k+1] == pc[k] + Δ * vc[k]);
        opti_ptr_->subject_to(vc[k+1] == vc[k] + Δ * ac[k]);

        // equality constraint
        casadi::MX diff = pc[k] - pcom[k];
        casadi::MX cross_expr = casadi::MX::cross(diff, fc[k]);
        opti_ptr_->subject_to(cross_expr == 0);

        // contact height constraint
        opti_ptr_->subject_to(pc[k](2) == 0.0);

        // inequality constraint
        opti_ptr_->subject_to(fc[k](2) >= 0.0);

        casadi::DM pcom_ref_DM = casadi::DM::zeros(3,1);
        casadi::DM zmp_ref_DM  = casadi::DM::zeros(3,1);
        for (int i = 0; i < 3; ++i){
          pcom_ref_DM(i) = static_cast<double>(pcom_ref(i, k));
          zmp_ref_DM(i) = static_cast<double>(zmp_ref(i, k));
        }
        casadi::DM fc_ref_DM  = casadi::DM::zeros(3,1);
        fc_ref_DM(2) = m*grav;
        
        // cost function
        cost += 40.0 * dot(pcom[k](0) - pcom_ref_DM(0), pcom[k](0) - pcom_ref_DM(0))    // 2
              + 40.0 * dot(pcom[k](1) - pcom_ref_DM(1), pcom[k](1) - pcom_ref_DM(1))    // 2
              + 50.0 * dot(pcom[k](2) - pcom_ref_DM(2), pcom[k](2) - pcom_ref_DM(2))    // 5
              + 0.01 * dot(vcom[k], vcom[k])                                            // 0.0001
              + 0.1 * dot(pc[k] - zmp_ref_DM, pc[k] - zmp_ref_DM)                       // 0.01
              + 0.00000001 * dot(vc[k], vc[k])
              + 1e-9 * dot(ac[k], ac[k])                                                //1e-7 
              + 1e-7 * dot(fc[k] - fc_ref_DM, fc[k] - fc_ref_DM);                       //1e-7 
    }


    casadi::DM pcom_ref_DM_ter = casadi::DM::zeros(3,1);
    casadi::DM zmp_ref_DM_ter  = casadi::DM::zeros(3,1);
    for (int i = 0; i < 3; ++i){
      pcom_ref_DM_ter(i) = static_cast<double>(pcom_ref(i, NH));
      zmp_ref_DM_ter(i) = static_cast<double>(zmp_ref(i, NH));
    }
    // terminal cost 
     cost += 15.0 * dot(pcom[NH] - pcom_ref_DM_ter, pcom[NH] - pcom_ref_DM_ter)
            + 0.0001 * dot(vcom[NH], vcom[NH])
            + 0.0001 * dot(pc[NH] - zmp_ref_DM_ter, pc[NH] - zmp_ref_DM_ter)
            + 0.00000001 * dot(vc[NH], vc[NH]);


    // terminal constraint
    opti_ptr_->subject_to(pcom[NH](0) == pc[NH](0));
    opti_ptr_->subject_to(pcom[NH](1) == pc[NH](1));


    opti_ptr_->minimize(cost);

    // set initial conditions
    p_x0 = opti_ptr_->parameter(12, 1);
    opti_ptr_->subject_to(pcom[0] == p_x0(casadi::Slice(0,3)));
    opti_ptr_->subject_to(vcom[0] == p_x0(casadi::Slice(3,6)));
    opti_ptr_->subject_to(pc[0]   == p_x0(casadi::Slice(6,9)));
    opti_ptr_->subject_to(vc[0]   == p_x0(casadi::Slice(9,12)));

  };

  
  SolutionMPC get_solution() const {
    return {
      {pos_com_, vel_com_, acc_com_},   // COM
      {pos_zmp_, vel_zmp_, acc_zmp_}    // ZMP
    };
  }
  

  void set_reference_trajectory(Eigen::Matrix<double, NY, NH+1>& traj_ref){
    // zmp_ref = traj_ref;
  }


  void solve(Eigen::Vector<double, NX> x0);

  void set_warm_start(const casadi::OptiSol& sol, const casadi::DM& p0);

  bool record_logs = false;
  double t_msec = 0.0;

private:
  Eigen::Vector3d pos_com_, vel_com_, acc_com_, pos_zmp_, vel_zmp_, acc_zmp_;

  // parameters
  double grav = 9.81;                 // gravity
  double Δ = 0.002;                   // time step

  double m = 44.0763;


  // zmp ref trajectory
  Eigen::Matrix<double, NY, NH+1> pcom_ref = Eigen::Matrix<double, NY, NH + 1>::Zero();
  Eigen::Matrix<double, NY, NH+1> zmp_ref = Eigen::Matrix<double, NY, NH + 1>::Zero();

  // casadi set up
  casadi::DM g = casadi::DM({0, 0, -grav});
  casadi::MX p_x0;
  std::vector<casadi::MX> state_vars_;
  std::vector<casadi::MX> input_vars_;
  std::shared_ptr<casadi::Opti> opti_ptr_;

}; 

} // end namespace labrob
