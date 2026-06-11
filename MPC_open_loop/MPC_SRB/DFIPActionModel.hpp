#pragma once 

#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/solvers/intro.hpp>
#include <crocoddyl/core/solvers/ipopt.hpp>
#include <crocoddyl/core/solvers/kkt.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/states/euclidean.hpp>

#include <Eigen/Dense>


#include <pinocchio/algorithm/centroidal.hpp>     // to use pinocchio::skew
using pinocchio::skew;

using namespace crocoddyl;


class DFIPActionModel : public ActionModelAbstractTpl<double> {
public:
  typedef ActionModelAbstractTpl<double> Base;
  typedef ActionDataAbstractTpl<double>  Data;
  typedef StateVectorTpl<double>         StateVector;

  explicit DFIPActionModel(int NX, int NU, double dt, double d_off, double m, Eigen::Matrix3d& I)
  : Base(std::make_shared<StateVector>(NX),  // nx, ndx (StateVector uses nx==ndx)
        NU),                                 // nu,     nr=0 for default
        NX_(NX),
        NU_(NU),
        dt_(dt),
        d_off_(d_off),
        m_(m),
        I_(I)
  {


    // default weights
    w_pcomxy_k_  = 8000.0;
    w_pcomz_k_   = 1000.0;
    w_vcomxy_k_  = 20.0;     
    w_vcomz_k_   = 20.0; 

    w_roll_k_    = 1000.0;
    w_pitch_k_   = 1000.0;
    w_yaw_k_     = 1000.0;

    w_ang_vel_k_ = 0.0;

    w_vr_k_      = 0.0;
    w_vrz_k_     = 0.0;
    w_vl_k_      = 0.0;
    w_vlz_k_     = 0.0;

    w_ar_k_      = 0.0001;
    w_arz_k_     = 0.001;
    w_al_k_      = 0.0001;
    w_alz_k_     = 0.001;

    w_fcxy_k_     = 0.000001;
    w_fcz_k_      = 0.00001;
    w_eq_k_       = 1000000.0;        // 1000000;

    grav = Eigen::Vector3d(0,0,-9.81);
    I_inv_ = I_.inverse();
  }


  // ---- required by CROCODDYL_BASE_CAST on ActionModelBase ----
  std::shared_ptr<ActionModelBase> cloneAsDouble() const override {
    return std::allocate_shared<DFIPActionModel>(
        Eigen::aligned_allocator<DFIPActionModel>(), *this);
  }
  std::shared_ptr<ActionModelBase> cloneAsFloat() const override {
    return std::allocate_shared<DFIPActionModel>(
        Eigen::aligned_allocator<DFIPActionModel>(), *this);
  }
  // ------------------------------------------------------------

  void setReference(const Eigen::VectorXd& x_ref, const Eigen::VectorXd& u_ref) {
    // checkJumpState(x_ref);
    x_ref_k_ = x_ref;
    u_ref_k_ = u_ref;
  }


  void setReference(const Eigen::VectorXd& x_ref) {
    // checkJumpState(x_ref);
    x_ref_k_ = x_ref;
    u_ref_k_.resize(0);
  }

  // override Running model ---- dynamics + cost ----
  void calc(const std::shared_ptr<Data>& data,
            const Eigen::Ref<const Eigen::VectorXd>& x,
            const Eigen::Ref<const Eigen::VectorXd>& u) override {
    Eigen::Vector3d pcom    = x.segment<3>(0);
    Eigen::Vector3d vcom    = x.segment<3>(3);
    Eigen::Vector3d rpy     = x.segment<3>(6);
    double& roll            = rpy(0);
    double& pitch           = rpy(1);
    double& yaw             = rpy(2);
    Eigen::Vector3d ang_vel = x.segment<3>(9);
    Eigen::Vector3d pr    = x.segment<3>(12);
    double vr              = x(15);
    double vr_z            = x(16);
    Eigen::Vector3d pl    = x.segment<3>(17);
    double vl              = x(20);
    double vl_z            = x(21);

    double ar              = u(0);
    double ar_z            = u(1);
    double al              = u(2);
    double al_z            = u(3);
    Eigen::Vector3d fr      = u.segment<3>(4);
    Eigen::Vector3d fl      = u.segment<3>(7);


    // dynamics
    data->xnext.segment<3>(0) = pcom + dt_ * vcom;
    data->xnext.segment<3>(3) = vcom + dt_ * ((fl + fr) / m_ + grav);
    data->xnext.segment<3>(6) = rpy + dt_ * ang_vel;                    // <----- small-angle approximation  (rpy_dot ≈ omega)
    data->xnext.segment<3>(9) = ang_vel + dt_ * (I_inv_ * (((pr - pcom).cross(fr) + (pl - pcom).cross(fl)) - ang_vel.cross((I_ * ang_vel))));
    data->xnext(12)  = pr(0) + dt_ * (vr * cos(yaw));
    data->xnext(13)  = pr(1) + dt_ * (vr * sin(yaw));
    data->xnext(14)  = pr(2) + dt_ * vr_z;
    data->xnext(15)  = vr + dt_ * ar;
    data->xnext(16)  = vr_z + dt_ * ar_z;
    data->xnext(17)  = pl(0) + dt_ * (vl * cos(yaw));
    data->xnext(18)  = pl(1) + dt_ * (vl * sin(yaw));
    data->xnext(19)  = pl(2) + dt_ * vl_z;
    data->xnext(20)  = vl + dt_ * al;
    data->xnext(21)  = vl_z + dt_ * al_z;


    
    double running_cost = 0.0;
    running_cost = 0.5 * w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0)).squaredNorm()
                 + 0.5 * w_pcomz_k_  * (pcom(2) - x_ref_k_(2)) * (pcom(2) - x_ref_k_(2))
                 + 0.5 * w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3)).squaredNorm()
                 + 0.5 * w_vcomz_k_  * (vcom(2) - x_ref_k_(5)) * (vcom(2) - x_ref_k_(5)) 
                 + 0.5 * w_roll_k_   * (roll - x_ref_k_(6)) * (roll - x_ref_k_(6))
                 + 0.5 * w_pitch_k_  * (pitch - x_ref_k_(7)) * (pitch - x_ref_k_(7))
                 + 0.5 * w_yaw_k_    * (yaw - x_ref_k_(8)) * (yaw - x_ref_k_(8))

                 + 0.5 * w_ang_vel_k_ * (ang_vel - x_ref_k_.segment<3>(9)).squaredNorm()

                 + 0.5 * w_vr_k_ * (vr - x_ref_k_(12)) * (vr - x_ref_k_(12))
                 + 0.5 * w_vrz_k_ * (vr_z - x_ref_k_(13)) * (vr_z - x_ref_k_(13))
                 + 0.5 * w_vl_k_ * (vl - x_ref_k_(14)) * (vl - x_ref_k_(14))
                 + 0.5 * w_vlz_k_ * (vl_z - x_ref_k_(15)) * (vl_z - x_ref_k_(15))
                 
              
                 + 0.5 * w_ar_k_ * (ar - u_ref_k_(0)) * (ar - u_ref_k_(0)) 
                 + 0.5 * w_arz_k_ * (ar_z - u_ref_k_(1)) * (ar_z - u_ref_k_(1)) 
                 + 0.5 * w_al_k_ * (al - u_ref_k_(2)) * (al - u_ref_k_(2)) 
                 + 0.5 * w_alz_k_ * (al_z - u_ref_k_(3)) * (al_z - u_ref_k_(3)) 

                 + 0.5 * w_fcxy_k_ * (fr.segment<2>(0) - u_ref_k_.segment<2>(4)).squaredNorm()
                 + 0.5 * w_fcz_k_ * (fr(2) - u_ref_k_(6)) * (fr(2) - u_ref_k_(6))
                 + 0.5 * w_fcxy_k_ * (fl.segment<2>(0) - u_ref_k_.segment<2>(7)).squaredNorm()
                 + 0.5 * w_fcz_k_ * (fl(2) - u_ref_k_(9)) * (fl(2) - u_ref_k_(9));
    

    double h_contact_r = vr_z - 0.0;
    double h_contact_l = vl_z - 0.0;
    
    running_cost += + 0.5 * w_eq_k_* h_contact_r * h_contact_r
                    + 0.5 * w_eq_k_* h_contact_l * h_contact_l;
    

    data->cost = running_cost;
  }


  // override for terminal model
  void calc(const std::shared_ptr<Data>& data,
            const Eigen::Ref<const Eigen::VectorXd>& x) override {

    Eigen::Vector3d pcom    = x.segment<3>(0);
    Eigen::Vector3d vcom    = x.segment<3>(3);
    Eigen::Vector3d rpy     = x.segment<3>(6);
    double& roll            = rpy(0);
    double& pitch           = rpy(1);
    double& yaw             = rpy(2);
    Eigen::Vector3d ang_vel = x.segment<3>(9);
    Eigen::Vector3d pr    = x.segment<3>(12);
    double vr              = x(15);
    double vr_z            = x(16);
    Eigen::Vector3d pl    = x.segment<3>(17);
    double vl              = x(20);
    double vl_z            = x(21);

    
    double running_cost = 0.0;
    running_cost = 0.5 * w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0)).squaredNorm()
                 + 0.5 * w_pcomz_k_  * (pcom(2) - x_ref_k_(2)) * (pcom(2) - x_ref_k_(2))
                 + 0.5 * w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3)).squaredNorm()
                 + 0.5 * w_vcomz_k_  * (vcom(2) - x_ref_k_(5)) * (vcom(2) - x_ref_k_(5)) 
                 + 0.5 * w_roll_k_   * (roll - x_ref_k_(6)) * (roll - x_ref_k_(6))
                 + 0.5 * w_pitch_k_  * (pitch - x_ref_k_(7)) * (pitch - x_ref_k_(7))
                 + 0.5 * w_yaw_k_    * (yaw - x_ref_k_(8)) * (yaw - x_ref_k_(8))

                 + 0.5 * w_ang_vel_k_ * (ang_vel - x_ref_k_.segment<3>(9)).squaredNorm()

                 + 0.5 * w_vr_k_ * (vr - x_ref_k_(12)) * (vr - x_ref_k_(12))
                 + 0.5 * w_vrz_k_ * (vr_z - x_ref_k_(13)) * (vr_z - x_ref_k_(13))
                 + 0.5 * w_vl_k_ * (vl - x_ref_k_(14)) * (vl - x_ref_k_(14))
                 + 0.5 * w_vlz_k_ * (vl_z - x_ref_k_(15)) * (vl_z - x_ref_k_(15));
  
    // contact constraint
    double h_contact_r = vr_z - 0.0;
    double h_contact_l = vl_z - 0.0;


    Eigen::Matrix2d R_yaw;
    R_yaw << std::cos(yaw), -std::sin(yaw),
            std::sin(yaw),  std::cos(yaw);
    
    // check
    double h_stability_r = R_yaw.transpose().row(0) * (pcom.segment<2>(0) - pr.segment<2>(0));
    double h_stability_l = R_yaw.transpose().row(0) * (pcom.segment<2>(0) - pl.segment<2>(0));

    running_cost += + 0.5 * w_eq_k_* h_contact_r * h_contact_r
                    + 0.5 * w_eq_k_* h_contact_l * h_contact_l
                    + 0.5 * w_eq_k_ * h_stability_r * h_stability_r
                    + 0.5 * w_eq_k_ * h_stability_l * h_stability_l;

    data->cost = running_cost;
  }

  // override Running model
  void calcDiff(const std::shared_ptr<Data>& data,
                const Eigen::Ref<const Eigen::VectorXd>& x,
                const Eigen::Ref<const Eigen::VectorXd>& u) override {
    
    Eigen::Vector3d pcom    = x.segment<3>(0);
    Eigen::Vector3d vcom    = x.segment<3>(3);
    Eigen::Vector3d rpy     = x.segment<3>(6);
    double& roll            = rpy(0);
    double& pitch           = rpy(1);
    double& yaw             = rpy(2);
    Eigen::Vector3d ang_vel = x.segment<3>(9);
    Eigen::Vector3d pr    = x.segment<3>(12);
    double vr              = x(15);
    double vr_z            = x(16);
    Eigen::Vector3d pl    = x.segment<3>(17);
    double vl              = x(20);
    double vl_z            = x(21);

    double ar              = u(0);
    double ar_z            = u(1);
    double al              = u(2);
    double al_z            = u(3);
    Eigen::Vector3d fr      = u.segment<3>(4);
    Eigen::Vector3d fl      = u.segment<3>(7);

    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);                        

    // Lx
    data->Lx.setZero();
    data->Lx.segment<2>(0) = w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0));
    data->Lx(2) = w_pcomz_k_ * (pcom(2) - x_ref_k_(2));
    data->Lx.segment<2>(3) = w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3));
    data->Lx(5) = w_vcomz_k_ * (vcom(2) - x_ref_k_(5));

    data->Lx(6) = w_roll_k_  * (roll - x_ref_k_(6));
    data->Lx(7) = w_pitch_k_ * (pitch - x_ref_k_(7));
    data->Lx(8) = w_yaw_k_   * (yaw - x_ref_k_(8));

    data->Lx.segment<3>(9) = w_ang_vel_k_ * (ang_vel - x_ref_k_.segment<3>(9));

    data->Lx(15) =  w_vr_k_ * (vr - x_ref_k_(12));
    data->Lx(16) =  w_vrz_k_ * (vr_z - x_ref_k_(13));
    data->Lx(20) =  w_vl_k_ * (vl - x_ref_k_(14));
    data->Lx(21) =  w_vlz_k_ * (vl_z - x_ref_k_(15));
  

    // Lxx
    data->Lxx.setZero();
    data->Lxx.block<2,2>(0,0) = w_pcomxy_k_ * I2;
    data->Lxx(2,2) = w_pcomz_k_;
    data->Lxx.block<2,2>(3,3) = w_vcomxy_k_ * I2;
    data->Lxx(5,5) = w_vcomz_k_;

    data->Lxx(6,6) = w_roll_k_;
    data->Lxx(7,7) = w_pitch_k_;
    data->Lxx(8,8) = w_yaw_k_;

    data->Lxx.block<3,3>(9,9) = w_ang_vel_k_ * I3;

    data->Lxx(15,15) =  w_vr_k_;
    data->Lxx(16,16) =  w_vrz_k_;
    data->Lxx(20,20) =  w_vl_k_;
    data->Lxx(21,21) =  w_vlz_k_;


    // Lxu
    data->Lxu.setZero();
  
    // Lu
    data->Lu.setZero();
    data->Lu(0) = w_ar_k_ * (ar - u_ref_k_(0));
    data->Lu(1) = w_arz_k_ * (ar_z - u_ref_k_(1));
    data->Lu(2) = w_al_k_ * (al - u_ref_k_(2));
    data->Lu(3) = w_alz_k_ * (al_z - u_ref_k_(3));

    data->Lu.segment<2>(4) = w_fcxy_k_ * (fr.segment<2>(0) - u_ref_k_.segment<2>(4));
    data->Lu(6) = w_fcz_k_ * (fr(2) - u_ref_k_(6));
    data->Lu.segment<2>(7) = w_fcxy_k_ * (fl.segment<2>(0) - u_ref_k_.segment<2>(7));
    data->Lu(9) = w_fcz_k_ * (fl(2) - u_ref_k_(9));
    
    // Luu
    data->Luu.setZero();
    data->Luu(0,0) = w_ar_k_;
    data->Luu(1,1) = w_arz_k_;
    data->Luu(2,2) = w_al_k_;
    data->Luu(3,3) = w_alz_k_;

    data->Luu.block<2,2>(4,4) = w_fcxy_k_ * I2;
    data->Luu(6,6) = w_fcz_k_;
    data->Luu.block<2,2>(7,7) = w_fcxy_k_ * I2;
    data->Luu(9,9) = w_fcz_k_;
    
    // contact constraints
    double h_contact_r = vr_z - 0.0;
    double h_contact_l = vl_z - 0.0;

    // Lx
    data->Lx(16) += w_eq_k_* h_contact_r;
    data->Lx(21) += w_eq_k_* h_contact_l;

    // Lxx
    data->Lxx(16, 16) += w_eq_k_;
    data->Lxx(21, 21) += w_eq_k_;
    

    
    // dynamics 
    data->Fx.setZero();
    data->Fx = Eigen::MatrixXd::Identity(NX_, NX_);
    data->Fx.block<3,3>(0,3) = dt_ * I3;
    data->Fx.block<3,3>(6,9) = dt_ * I3;

    // dw/dpcom
    data->Fx.block<3,3>(9,0) = dt_ * I_inv_ * (skew(fl) + skew(fr));

    // dw/dw
    data->Fx.block<3,3>(9,9) += dt_ * I_inv_ * (skew(I_ * ang_vel) - skew(ang_vel) * I_);

    // dw/dpr
    data->Fx.block<3,3>(9,12) = - dt_ * I_inv_ * skew(fr);

    // dw/dpr
    data->Fx.block<3,3>(9,17) = - dt_ * I_inv_ * skew(fl);
                  
    // dpr/dyaw
    data->Fx(12,8) = - dt_ * vr * sin(yaw);
    data->Fx(13,8) = dt_ * vr * cos(yaw);

    data->Fx(12,15) = dt_ * cos(yaw);
    data->Fx(13,15) = dt_ * sin(yaw);
    data->Fx(14,16) = dt_;

    // dpl/dyaw
    data->Fx(17,8) = - dt_ * vl * sin(yaw);
    data->Fx(18,8) = dt_ * vl * cos(yaw);

    data->Fx(17,20) = dt_ * cos(yaw);
    data->Fx(18,20) = dt_ * sin(yaw);
    data->Fx(19,21) = dt_;

    data->Fu.setZero();
    data->Fu.block<3,3>(3,4) = dt_ * 1/m_ * I3;
    data->Fu.block<3,3>(3,7) = dt_ * 1/m_ * I3;
    data->Fu.block<3,3>(9,4) = dt_ * I_inv_ * skew(pr - pcom);
    data->Fu.block<3,3>(9,7) = dt_ * I_inv_ * skew(pl - pcom);
    data->Fu(15, 0) = dt_;
    data->Fu(16, 1) = dt_;
    data->Fu(20, 2) = dt_;
    data->Fu(21, 3) = dt_;
}



  // override Terminal model
  void calcDiff(const std::shared_ptr<Data>& data,
                  const Eigen::Ref<const Eigen::VectorXd>& x) override {
    
    Eigen::Vector3d pcom    = x.segment<3>(0);
    Eigen::Vector3d vcom    = x.segment<3>(3);
    Eigen::Vector3d rpy     = x.segment<3>(6);
    double& roll            = rpy(0);
    double& pitch           = rpy(1);
    double& yaw             = rpy(2);
    Eigen::Vector3d ang_vel = x.segment<3>(9);
    Eigen::Vector3d pr    = x.segment<3>(12);
    double vr              = x(15);
    double vr_z            = x(16);
    Eigen::Vector3d pl    = x.segment<3>(17);
    double vl              = x(20);
    double vl_z            = x(21);
    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);                        

    // Lx
    data->Lx.setZero();
    data->Lx.segment<2>(0) = w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0));
    data->Lx(2) = w_pcomz_k_ * (pcom(2) - x_ref_k_(2));
    data->Lx.segment<2>(3) = w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3));
    data->Lx(5) = w_vcomz_k_ * (vcom(2) - x_ref_k_(5));

    data->Lx(6) = w_roll_k_  * (roll - x_ref_k_(6));
    data->Lx(7) = w_pitch_k_ * (pitch - x_ref_k_(7));
    data->Lx(8) = w_yaw_k_   * (yaw - x_ref_k_(8));

    data->Lx.segment<3>(9) = w_ang_vel_k_ * (ang_vel - x_ref_k_.segment<3>(9));

    data->Lx(15) =  w_vr_k_ * (vr - x_ref_k_(12));
    data->Lx(16) =  w_vrz_k_ * (vr_z - x_ref_k_(13));
    data->Lx(20) =  w_vl_k_ * (vl - x_ref_k_(14));
    data->Lx(21) =  w_vlz_k_ * (vl_z - x_ref_k_(15));
  

    // Lxx
    data->Lxx.setZero();
    data->Lxx.block<2,2>(0,0) = w_pcomxy_k_ * I2;
    data->Lxx(2,2) = w_pcomz_k_;
    data->Lxx.block<2,2>(3,3) = w_vcomxy_k_ * I2;
    data->Lxx(5,5) = w_vcomz_k_;

    data->Lxx(6,6) = w_roll_k_;
    data->Lxx(7,7) = w_pitch_k_;
    data->Lxx(8,8) = w_yaw_k_;

    data->Lxx.block<3,3>(9,9) = w_ang_vel_k_ * I3;

    data->Lxx(15,15) =  w_vr_k_;
    data->Lxx(16,16) =  w_vrz_k_;
    data->Lxx(20,20) =  w_vl_k_;
    data->Lxx(21,21) =  w_vlz_k_;



    // contact constraints
    double h_contact_r = vr_z - 0.0;
    double h_contact_l = vl_z - 0.0;

    Eigen::Matrix2d R_yaw;
    R_yaw << std::cos(yaw), -std::sin(yaw),
            std::sin(yaw),  std::cos(yaw);

    // stability constraints
    double h_stability_r = R_yaw.transpose().row(0) * (pcom.segment<2>(0) - pr.segment<2>(0));
    double h_stability_l = R_yaw.transpose().row(0) * (pcom.segment<2>(0) - pl.segment<2>(0));

    Eigen::Vector2d ones_2 = Eigen::Vector2d(1,1);

    Eigen::MatrixXd Jx_stability_r = Eigen::MatrixXd::Zero(1, NX_);
    Jx_stability_r(0,0) = std::cos(yaw);
    Jx_stability_r(0,1) = std::sin(yaw);
    Jx_stability_r(0,8) = - std::sin(yaw) * (pcom.x() - pr.x()) + std::cos(yaw) * (pcom.y() - pr.y());
    Jx_stability_r(0,12) = - std::cos(yaw);
    Jx_stability_r(0,13) = - std::sin(yaw);

    Eigen::MatrixXd Jx_stability_l = Eigen::MatrixXd::Zero(1, NX_);
    Jx_stability_l(0,0) = std::cos(yaw);
    Jx_stability_l(0,1) = std::sin(yaw);
    Jx_stability_l(0,8) = - std::sin(yaw) * (pcom.x() - pl.x()) + std::cos(yaw) * (pcom.y() - pl.y());
    Jx_stability_l(0,17) = - std::cos(yaw);
    Jx_stability_l(0,18) = - std::sin(yaw);

    // Lx
    data->Lx(16) += w_eq_k_* h_contact_r;
    data->Lx(21) += w_eq_k_* h_contact_l;

    data->Lx += w_eq_k_ * Jx_stability_r.transpose() * h_stability_r;
    data->Lx += w_eq_k_ * Jx_stability_l.transpose() * h_stability_l;


    // Lxx
    data->Lxx(16, 16) += w_eq_k_;
    data->Lxx(21, 21) += w_eq_k_;
    
    data->Lxx += w_eq_k_ * Jx_stability_r.transpose() * Jx_stability_r;
    data->Lxx += w_eq_k_ * Jx_stability_l.transpose() * Jx_stability_l;

    
  }


  Eigen::VectorXd x_ref_k_;
  Eigen::VectorXd u_ref_k_;
  bool jump_state = false;

private:
  // weights
  double w_pcomxy_k_;
  double w_pcomz_k_;
  double w_vcomxy_k_;     
  double w_vcomz_k_; 

  double w_roll_k_;
  double w_pitch_k_;
  double w_yaw_k_;

  double w_ang_vel_k_;

  double w_vr_k_;
  double w_vrz_k_;
  double w_vl_k_;
  double w_vlz_k_;

  double w_ar_k_;
  double w_arz_k_;
  double w_al_k_;
  double w_alz_k_;

  double w_fcxy_k_;
  double w_fcz_k_;
  double w_eq_k_;

  const int NX_;
  const int NU_;
  double dt_;

  double d_off_ = 0.1;
  double m_     = 27;
  Eigen::Matrix3d I_ = Eigen::Matrix3d::Identity();   // moment Inertia of the rigid body
  Eigen::Matrix3d I_inv_;
  Eigen::Vector3d grav;
};