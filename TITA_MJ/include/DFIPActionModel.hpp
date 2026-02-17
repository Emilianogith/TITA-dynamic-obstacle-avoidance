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

  explicit DFIPActionModel(int NX, int NU, double dt, double d_off, double m)
  : Base(std::make_shared<StateVector>(NX),  // nx, ndx (StateVector uses nx==ndx)
        NU),                                 // nu,     nr=0 for default
        NX_(NX),
        NU_(NU),
        dt_(dt),
        d_off_(d_off),
        m_(m)
  {
    // default weights
    w_pcomxy_k_ = 10.0;  
    w_pcomz_k_  = 1000.0;
    w_vcomxy_k_ = 10.0; 
    w_vcomz_k_  = 50.0;
    w_c_k_      = 0.0; 
    w_v_k_      = 0.0;
    w_vcz_k_    = 0.0;

    w_theta_k_  = 0.0;
    w_w_k_      = 5.0;

    w_a_k_      = 0.1;
    w_acz_k_    = 0.0;
    
    w_alpha_k_  = 0.001;

    w_fcxy_k_   = 0.00001;       // 0.0000001;
    w_fcz_k_    = 0.0001;          // 0.0000001;

    w_eq_k_     = 1000000.0;     // 100000000;

    x_ref_k_.setZero(NX_);
    u_ref_k_.setZero(NU_);

    grav = Eigen::Vector3d(0,0,-9.81);
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


   void checkJumpState(const Eigen::VectorXd& x_ref) {
    if (std::fabs(x_ref(9)) > 1e-3) {
      jump_state = true;  // check if desired contact height is greater than 0.0

      w_pcomxy_k_ = 10.0;
      w_pcomz_k_  = 1000.0;
      w_vcomxy_k_ = 10.0;
      w_vcomz_k_  = 100.0;
      w_c_k_      = 0.0; 
      w_v_k_      = 100.0;
      w_vcz_k_    = 10.0;
              
      w_theta_k_  = 0.0;
      w_w_k_      = 0.0;

      w_a_k_      = 1e-6;
      w_acz_k_    = 0.000001;
      
      w_alpha_k_  = 1e-6;

      w_fcxy_k_   = 1e-6;
      w_fcz_k_    = 1e-6;

      // w_eq_k_     = 0.0;

    } else if (jump_state){
      jump_state = false;

      // default weights
      w_pcomxy_k_ = 10.0;
      w_pcomz_k_  = 1000.0;
      w_vcomxy_k_ = 10.0;
      w_vcomz_k_  = 100.0;
      w_c_k_      = 0.0; 
      w_v_k_      = 0.0;
      w_vcz_k_    = 0.0;

      w_theta_k_  = 0.0;
      w_w_k_      = 5.0;

      w_a_k_      = 0.1;
      w_acz_k_    = 0.0;
      
      w_alpha_k_  = 0.001;

      w_fcxy_k_   = 0.0000001;
      w_fcz_k_    = 0.0001;
    }
  }

  void setReference(const Eigen::VectorXd& x_ref, const Eigen::VectorXd& u_ref) {
    checkJumpState(x_ref);
    x_ref_k_ = x_ref;
    u_ref_k_ = u_ref;
  }


  void setReference(const Eigen::VectorXd& x_ref) {
    checkJumpState(x_ref);
    x_ref_k_ = x_ref;
    u_ref_k_.resize(0);
  }

  // override Running model ---- dynamics + cost ----
  void calc(const std::shared_ptr<Data>& data,
            const Eigen::Ref<const Eigen::VectorXd>& x,
            const Eigen::Ref<const Eigen::VectorXd>& u) override {
    Eigen::Vector3d pcom = x.segment<3>(0);
    Eigen::Vector3d vcom = x.segment<3>(3);
    Eigen::Vector3d c = x.segment<3>(6);
    double vc_z   = x(9);
    double theta = x(10);
    double v     = x(11);
    double w     = x(12);
    double a     = u(0);
    double ac_z  = u(1);
    double alpha = u(2);
    Eigen::Vector3d fl = u.segment<3>(3);
    Eigen::Vector3d fr = u.segment<3>(6);


    // force contact point construction
    Eigen::Vector3d vector_off = Eigen::Vector3d(0.0, d_off_/2, 0.0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R << cos(theta), -sin(theta), 0,
         sin(theta), cos(theta),  0,
         0, 0, 1;
    Eigen::Vector3d pl = c + R * vector_off;
    Eigen::Vector3d pr = c - R * vector_off;


    // dynamics
    data->xnext.segment<3>(0) = pcom + dt_ * vcom;
    data->xnext.segment<3>(3) = vcom + dt_ * ((fl + fr) / m_ + grav);
    data->xnext(6)  = c(0) + dt_ * (v * cos(theta));
    data->xnext(7)  = c(1) + dt_ * (v * sin(theta));
    data->xnext(8)  = c(2) + dt_ * vc_z;
    data->xnext(9)  = vc_z + dt_ * ac_z;
    data->xnext(10) = theta + dt_ * w;
    data->xnext(11) = v + dt_ * a;
    data->xnext(12) = w + dt_ * alpha;

    
    double running_cost = 0.0;
    running_cost = 0.5 * w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0)).squaredNorm()
    + 0.5 * w_pcomz_k_ * (pcom(2) - x_ref_k_(2)) * (pcom(2) - x_ref_k_(2))
    + 0.5 * w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3)).squaredNorm()
    + 0.5 * w_vcomz_k_ * (vcom(2) - x_ref_k_(5)) * (vcom(2) - x_ref_k_(5)) 
    + 0.5 * w_c_k_ *(c - x_ref_k_.segment<3>(6)).squaredNorm()
    + 0.5 * w_vcz_k_ * (vc_z - x_ref_k_(9)) * (vc_z - x_ref_k_(9))
    + 0.5 * w_theta_k_ * (theta - x_ref_k_(10)) * (theta - x_ref_k_(10))
    + 0.5 * w_v_k_ * (v - x_ref_k_(11)) * (v - x_ref_k_(11))
    + 0.5 * w_w_k_ * (w - x_ref_k_(12)) * (w - x_ref_k_(12))
    + 0.5 * w_a_k_ * (a - u_ref_k_(0)) * (a - u_ref_k_(0)) 
    + 0.5 * w_acz_k_ * (ac_z - u_ref_k_(1)) * (ac_z - u_ref_k_(1)) 
    + 0.5 * w_alpha_k_ * (alpha - u_ref_k_(2)) * (alpha - u_ref_k_(2)) 
    + 0.5 * w_fcxy_k_ * (fl.segment<2>(0) - u_ref_k_.segment<2>(3)).squaredNorm()
    + 0.5 * w_fcz_k_ * (fl(2) - u_ref_k_(5)) * (fl(2) - u_ref_k_(5))
    + 0.5 * w_fcxy_k_ * (fr.segment<2>(0) - u_ref_k_.segment<2>(6)).squaredNorm()
    + 0.5 * w_fcz_k_ * (fr(2) - u_ref_k_(8)) * (fr(2) - u_ref_k_(8));
    
    // build soft constraint
    if (!jump_state){
      double h_contact = vc_z - 0.0;
      Eigen::Vector3d h_moment = (pl - pcom).cross(fl) + (pr - pcom).cross(fr); 
      running_cost += + 0.5 * w_eq_k_* h_contact * h_contact
                      + 0.5 * w_eq_k_* h_moment.transpose() * h_moment;
    }

    if (jump_state){
      Eigen::Vector3d h_fl = fl;
      Eigen::Vector3d h_fr = fr; 
      double h_omega = w; 

      running_cost += 0.5 * w_eq_k_* h_omega * h_omega
                    + 0.5 * w_eq_k_* h_fl.transpose() * h_fl
                    + 0.5 * w_eq_k_* h_fr.transpose() * h_fr;
    }
    

    data->cost = running_cost;
  }


  // override for terminal model
  void calc(const std::shared_ptr<Data>& data,
            const Eigen::Ref<const Eigen::VectorXd>& x) override {
    Eigen::Vector3d pcom = x.segment<3>(0);
    Eigen::Vector3d vcom = x.segment<3>(3);
    Eigen::Vector3d c = x.segment<3>(6);
    double vc_z   = x(9);
    double theta = x(10);
    double v     = x(11);
    double w     = x(12);

    
    double running_cost = 0.0;
    running_cost = 0.5 * w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0)).squaredNorm()
    + 0.5 * w_pcomz_k_ * (pcom(2) - x_ref_k_(2)) * (pcom(2) - x_ref_k_(2))
    + 0.5 * w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3)).squaredNorm()
    + 0.5 * w_vcomz_k_ * (vcom(2) - x_ref_k_(5)) * (vcom(2) - x_ref_k_(5)) 
    + 0.5 * w_c_k_ *(c - x_ref_k_.segment<3>(6)).squaredNorm()
    + 0.5 * w_vcz_k_ * (vc_z - x_ref_k_(9)) * (vc_z - x_ref_k_(9))
    + 0.5 * w_theta_k_ * (theta - x_ref_k_(10)) * (theta - x_ref_k_(10))
    + 0.5 * w_v_k_ * (v - x_ref_k_(11)) * (v - x_ref_k_(11))
    + 0.5 * w_w_k_ * (w - x_ref_k_(12)) * (w - x_ref_k_(12));
    
    // build soft constraint
    if (!jump_state){
      double h_contact = vc_z - 0.0;
      Eigen::Vector2d h_stability = pcom.segment<2>(0) - c.segment<2>(0);
      running_cost += + 0.5 * w_eq_k_* h_contact * h_contact
                      + 0.5 * w_eq_k_ * h_stability.transpose() * h_stability;
    }

    if (jump_state){
      double h_omega = w; 
      running_cost += 0.5 * w_eq_k_* h_omega * h_omega;
    }

    data->cost = running_cost;
  }

  // override Running model
  void calcDiff(const std::shared_ptr<Data>& data,
                const Eigen::Ref<const Eigen::VectorXd>& x,
                const Eigen::Ref<const Eigen::VectorXd>& u) override {
    Eigen::Vector3d pcom = x.segment<3>(0);
    Eigen::Vector3d vcom = x.segment<3>(3);
    Eigen::Vector3d c = x.segment<3>(6);
    double vc_z   = x(9);
    double theta = x(10);
    double v     = x(11);
    double w     = x(12);
    double a     = u(0);
    double ac_z  = u(1);
    double alpha = u(2);
    Eigen::Vector3d fl = u.segment<3>(3);
    Eigen::Vector3d fr = u.segment<3>(6);


    // force contact point construction
    Eigen::Vector3d vector_off = Eigen::Vector3d(0.0, d_off_/2, 0.0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R << cos(theta), -sin(theta), 0,
         sin(theta), cos(theta),  0,
         0, 0, 1;
    Eigen::Matrix3d dR = Eigen::Matrix3d::Zero();
    dR << -sin(theta), -cos(theta), 0,
         cos(theta), -sin(theta),  0,
         0, 0, 0;
    Eigen::Vector3d pl = c + R * vector_off;
    Eigen::Vector3d pr = c - R * vector_off;

    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);                        

    // Lx
    data->Lx.setZero();
    data->Lx.segment<2>(0) = w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0));
    data->Lx(2) = w_pcomz_k_ * (pcom(2) - x_ref_k_(2));
    data->Lx.segment<2>(3) = w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3));
    data->Lx(5) = w_vcomz_k_ * (vcom(2) - x_ref_k_(5));
    data->Lx.segment<3>(6) = w_c_k_ * (c - x_ref_k_.segment<3>(6));
    data->Lx(9) = w_vcz_k_ * (vc_z - x_ref_k_(9));
    data->Lx(10) = w_theta_k_ * (theta - x_ref_k_(10));
    data->Lx(11) = w_v_k_ * (v - x_ref_k_(11));
    data->Lx(12) = w_w_k_ * (w - x_ref_k_(12));

    // Lxx
    data->Lxx.setZero();
    data->Lxx.block<2,2>(0,0) = w_pcomxy_k_ * I2;
    data->Lxx(2,2) = w_pcomz_k_;
    data->Lxx.block<2,2>(3,3) = w_vcomxy_k_ * I2;
    data->Lxx(5,5) = w_vcomz_k_;
    data->Lxx.block<3,3>(6,6) = w_c_k_ * I3;
    data->Lxx(9,9) = w_vcz_k_;
    data->Lxx(10,10) = w_theta_k_;
    data->Lxx(11,11) = w_v_k_;
    data->Lxx(12,12) = w_w_k_;


    // Lxu
    data->Lxu.setZero();
  
    // Lu
    data->Lu.setZero();
    data->Lu(0) = w_a_k_ * (a - u_ref_k_(0));
    data->Lu(1) = w_acz_k_ * (ac_z - u_ref_k_(1));
    data->Lu(2) = w_alpha_k_ * (alpha - u_ref_k_(2));
    data->Lu.segment<2>(3) = w_fcxy_k_ * (fl.segment<2>(0) - u_ref_k_.segment<2>(3));
    data->Lu(5) = w_fcz_k_ * (fl(2) - u_ref_k_(5));
    data->Lu.segment<2>(6) = w_fcxy_k_ * (fr.segment<2>(0) - u_ref_k_.segment<2>(6));
    data->Lu(8) = w_fcz_k_ * (fr(2) - u_ref_k_(8));
    
    // Luu
    data->Luu.setZero();
    data->Luu(0,0) = w_a_k_;
    data->Luu(1,1) = w_acz_k_;
    data->Luu(2,2) = w_alpha_k_;
    data->Luu.block<2,2>(3,3) = w_fcxy_k_ * I2;
    data->Luu(5,5) = w_fcz_k_;
    data->Luu.block<2,2>(6,6) = w_fcxy_k_ * I2;
    data->Luu(8,8) = w_fcz_k_;
    

    if (!jump_state){
      // z-contact constraint
      double h_contact = vc_z - 0.0;
      Eigen::MatrixXd Jx_contact = Eigen::MatrixXd::Zero(1, NX_);
      Jx_contact(0,9) = 1.0;
      Eigen::MatrixXd Ju_contact = Eigen::MatrixXd::Zero(1, NU_);

      // moment constraint
      Eigen::Vector3d h_moment = (pl - pcom).cross(fl) + (pr - pcom).cross(fr);
      Eigen::Matrix3d dh_dcom = skew(fl) + skew(fr);
      Eigen::Matrix3d dh_dc = -skew(fl) * I3 -skew(fr) * I3;
      Eigen::Vector3d dh_dtheta = -skew(fl) * dR * vector_off + skew(fr) * dR * vector_off;
      Eigen::Matrix3d dh_dfl = skew(pl - pcom);
      Eigen::Matrix3d dh_dfr = skew(pr - pcom);
      Eigen::MatrixXd Jx_moment = Eigen::MatrixXd::Zero(3, NX_);
      Eigen::MatrixXd Ju_moment = Eigen::MatrixXd::Zero(3, NU_);
      Jx_moment.block<3,3>(0, 0) = dh_dcom;
      Jx_moment.block<3,3>(0, 6) = dh_dc;
      Jx_moment.block<3,1>(0, 10) = dh_dtheta;
      Ju_moment.block<3,3>(0, 3) = dh_dfl;
      Ju_moment.block<3,3>(0, 6) = dh_dfr;

      data->Lx += w_eq_k_ * Jx_contact.transpose() * h_contact;
      data->Lxx += w_eq_k_ * Jx_contact.transpose() * Jx_contact;

      //moment constraint
      data->Lx += w_eq_k_ * Jx_moment.transpose() * h_moment;
      data->Lxx += w_eq_k_ * Jx_moment.transpose() * Jx_moment;
    
      data->Lxu += w_eq_k_ * Jx_moment.transpose() * Ju_moment;
      data->Lxu += w_eq_k_ * Jx_contact.transpose() * Ju_contact;

      data->Lu += w_eq_k_ * Ju_moment.transpose() * h_moment;
      data->Lu += w_eq_k_ * Ju_contact.transpose() * h_contact;

      data->Luu += w_eq_k_ * Ju_moment.transpose() * Ju_moment;
      data->Luu += w_eq_k_ * Ju_contact.transpose() * Ju_contact;
    }

    if (jump_state){
      double h_omega = w; 
      Eigen::MatrixXd Jx_omega = Eigen::MatrixXd::Zero(1, NX_);
      Jx_omega(0,12) = 1.0;

      Eigen::Vector3d h_fl = fl;
      Eigen::MatrixXd Ju_fl = Eigen::MatrixXd::Zero(3, NU_);
      Ju_fl.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

      Eigen::Vector3d h_fr = fr; 
      Eigen::MatrixXd Ju_fr = Eigen::MatrixXd::Zero(3, NU_);
      Ju_fr.block<3,3>(0, 6) = Eigen::Matrix3d::Identity();


      data->Lx += w_eq_k_ * Jx_omega.transpose() * h_omega;
      data->Lxx += w_eq_k_ * Jx_omega.transpose() * Jx_omega;

      data->Lu += w_eq_k_ * Ju_fl.transpose() * h_fl;
      data->Lu += w_eq_k_ * Ju_fr.transpose() * h_fr;
      
      data->Luu += w_eq_k_ * Ju_fl.transpose() * Ju_fl;
      data->Luu += w_eq_k_ * Ju_fr.transpose() * Ju_fr;
    }

    // dynamics 
    data->Fx.setZero();
    data->Fx = Eigen::MatrixXd::Identity(NX_, NX_);
    data->Fx.block<3,3>(0,3) = dt_ * I3;
    data->Fx(8, 9) = dt_;                         // c_z depends on vc_z
    data->Fx(10, 12) = dt_;                       // theta depends on w
    data->Fx(6, 10) = -dt_ * v * sin(theta);
    data->Fx(7, 10) =  dt_ * v * cos(theta);
    data->Fx(6, 11) =  dt_ * cos(theta);
    data->Fx(7, 11) =  dt_ * sin(theta);

    data->Fu.setZero();
    data->Fu.block<3,3>(3,3) = dt_ * 1/m_ * I3;
    data->Fu.block<3,3>(3,6) = dt_ * 1/m_ * I3;
    data->Fu(9, 1)  = dt_;
    data->Fu(11, 0) = dt_;
    data->Fu(12, 2) = dt_;
}



  // override Terminal model
  void calcDiff(const std::shared_ptr<Data>& data,
                  const Eigen::Ref<const Eigen::VectorXd>& x) override {
      Eigen::Vector3d pcom = x.segment<3>(0);
      Eigen::Vector3d vcom = x.segment<3>(3);
      Eigen::Vector3d c = x.segment<3>(6);
      double vc_z   = x(9);
      double theta = x(10);
      double v     = x(11);
      double w     = x(12);

      
      Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);

      // Lx
      data->Lx.setZero();
      data->Lx.segment<2>(0) = w_pcomxy_k_ * (pcom.segment<2>(0) - x_ref_k_.segment<2>(0));
      data->Lx(2) = w_pcomz_k_ * (pcom(2) - x_ref_k_(2));
      data->Lx.segment<2>(3) = w_vcomxy_k_ * (vcom.segment<2>(0) - x_ref_k_.segment<2>(3));
      data->Lx(5) = w_vcomz_k_ * (vcom(2) - x_ref_k_(5));
      data->Lx.segment<3>(6) = w_c_k_ * (c - x_ref_k_.segment<3>(6));
      data->Lx(9) = w_vcz_k_ * (vc_z - x_ref_k_(9));
      data->Lx(10) = w_theta_k_ * (theta - x_ref_k_(10));
      data->Lx(11) = w_v_k_ * (v - x_ref_k_(11));
      data->Lx(12) = w_w_k_ * (w - x_ref_k_(12));

      // Lxx
      data->Lxx.setZero();
      data->Lxx.block<2,2>(0,0) = w_pcomxy_k_ * I2;
      data->Lxx(2,2) = w_pcomz_k_;
      data->Lxx.block<2,2>(3,3) = w_vcomxy_k_ * I2;
      data->Lxx(5,5) = w_vcomz_k_;
      data->Lxx.block<3,3>(6,6) = w_c_k_ * I3;
      data->Lxx(9,9) = w_vcz_k_;
      data->Lxx(10,10) = w_theta_k_;
      data->Lxx(11,11) = w_v_k_;
      data->Lxx(12,12) = w_w_k_;


      if (!jump_state){
        // z-zmp constraint
        double h_contact = vc_z - 0.0;
        Eigen::MatrixXd Jx_contact = Eigen::MatrixXd::Zero(1, NX_);
        Jx_contact(0,9) = 1.0;

        data->Lx += w_eq_k_ * Jx_contact.transpose() * h_contact;
        data->Lxx += w_eq_k_ * Jx_contact.transpose() * Jx_contact;

        //stability constarint
        Eigen::Vector2d h_stability = pcom.segment<2>(0) - c.segment<2>(0);
        Eigen::MatrixXd Jx_stability = Eigen::MatrixXd::Zero(2, NX_);
        Jx_stability.block<2,2>(0,0) =  I2;
        Jx_stability.block<2,2>(0,6) = -I2;

        data->Lx += w_eq_k_ * Jx_stability.transpose() * h_stability;
        data->Lxx += w_eq_k_ * Jx_stability.transpose() * Jx_stability;

      }

      if (jump_state){
        double h_omega = w; 
        Eigen::MatrixXd Jx_omega = Eigen::MatrixXd::Zero(1, NX_);
        Jx_omega(0,12) = 1.0;

        data->Lx += w_eq_k_ * Jx_omega.transpose() * h_omega;
        data->Lxx += w_eq_k_ * Jx_omega.transpose() * Jx_omega;
      }
  }


  Eigen::VectorXd x_ref_k_;
  Eigen::VectorXd u_ref_k_;
  bool jump_state = false;

private:
  // weights
  double w_pcomz_k_;
  double w_pcomxy_k_;
  double w_vcomxy_k_;     
  double w_vcomz_k_; 

  double w_c_k_;
  double w_v_k_;
  double w_vcz_k_;
  double w_theta_k_;
  double w_w_k_;

  double w_a_k_;
  double w_acz_k_;
  double w_alpha_k_;
  double w_fcz_k_;
  double w_fcxy_k_;
  double w_eq_k_;

  const int NX_;
  const int NU_;
  double dt_;

  double d_off_ = 0.1;
  double m_     = 27;
  Eigen::Vector3d grav;
};