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


class UnicycleModel : public ActionModelAbstractTpl<double> {
public:
  typedef ActionModelAbstractTpl<double> Base;
  typedef ActionDataAbstractTpl<double>  Data;
  typedef StateVectorTpl<double>         StateVector;

  explicit UnicycleModel(int NX, int NU, double dt, double d_off, double m)
  : Base(std::make_shared<StateVector>(NX),  // nx, ndx (StateVector uses nx==ndx)
        NU),                                 // nu,     nr=0 for default
        NX_(NX),
        NU_(NU),
        dt_(dt),
        d_off_(d_off),
        m_(m)
  {
    // default weights
    w_pcomxy_k_ = 0.0;           // 10.0;
    w_pcomz_k_  = 0.0;           // 10.0;
    w_vcomxy_k_ = 0.0;           // 0.1;
    w_vcomz_k_  = 0.0;           // 0.1;
    w_c_k_      = 1.0;           // 0.01;
    w_v_k_      = 0.0;         // 0.001;

    w_theta_k_  = 1.0;           // 0.1;
    w_w_k_      = 0.0;        // 0.0001;

    w_a_k_      = 1e-6;        // 0.0001;
    
    w_alpha_k_  = 1e-6;        // 0.0001;

    w_fcxy_k_   = 0.0;     // 0.0000001;
    w_fcz_k_    = 0.0;     // 0.0000001;

    w_eq_k_     = 0.0;     // 100000000;

    x_ref_k_.setZero(NX_);
    u_ref_k_.setZero(NU_);

    grav = Eigen::Vector3d(0,0,-9.81);
  }


  // ---- required by CROCODDYL_BASE_CAST on ActionModelBase ----
  std::shared_ptr<ActionModelBase> cloneAsDouble() const override {
    return std::allocate_shared<UnicycleModel>(
        Eigen::aligned_allocator<UnicycleModel>(), *this);
  }
  std::shared_ptr<ActionModelBase> cloneAsFloat() const override {
    return std::allocate_shared<UnicycleModel>(
        Eigen::aligned_allocator<UnicycleModel>(), *this);
  }

  // struct ActionDataDI : public ActionDataAbstractTpl<double> {
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //   explicit ActionDataDI(UnicycleModel* model)
  //     : ActionDataAbstractTpl<double>(model) {}
  // };

  // std::shared_ptr<Data> createData() override {
  //   return std::allocate_shared<ActionDataDI>(
  //       Eigen::aligned_allocator<ActionDataDI>(), this);
  // }
  // ------------------------------------------------------------

  void setReference(const Eigen::VectorXd& x_ref, const Eigen::VectorXd& u_ref) {
    x_ref_k_ = x_ref;
    u_ref_k_ = u_ref;
  }


  void setReference(const Eigen::VectorXd& x_ref) {
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
    data->xnext.segment<3>(0).setZero();
    data->xnext.segment<3>(3).setZero();
    data->xnext(6)  = c(0) + dt_ * (v * cos(theta));
    data->xnext(7)  = c(1) + dt_ * (v * sin(theta));
    data->xnext(8)  = 0.0;
    data->xnext(9)  = 0.0;
    data->xnext(10) = theta + dt_ * w;
    data->xnext(11) = v + dt_ * a;
    data->xnext(12) = w + dt_ * alpha;

   
    
    double running_cost = 0.0;
    running_cost = 0.5 * w_c_k_ *(c - x_ref_k_.segment<3>(6)).squaredNorm()
                    + 0.5 * w_theta_k_ * (theta - x_ref_k_(10)) * (theta - x_ref_k_(10))
                    + 0.5 * w_v_k_ * (v - x_ref_k_(11)) * (v - x_ref_k_(11))
                    + 0.5 * w_w_k_ * (w - x_ref_k_(12)) * (w - x_ref_k_(12))
                    + 0.5 * w_a_k_ * (a - u_ref_k_(0)) * (a - u_ref_k_(0)) 
                    + 0.5 * w_alpha_k_ * (alpha - u_ref_k_(2)) * (alpha - u_ref_k_(2));
    

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


    // force contact point construction
    Eigen::Vector3d vector_off = Eigen::Vector3d(0.0, d_off_/2, 0.0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R << cos(theta), -sin(theta), 0,
         sin(theta), cos(theta),  0,
         0, 0, 1;
    Eigen::Vector3d pl = c + R * vector_off;
    Eigen::Vector3d pr = c - R * vector_off;


    double running_cost = 0.0;

    running_cost = 0.5 * w_c_k_ *(c - x_ref_k_.segment<3>(6)).squaredNorm()
                    + 0.5 * w_theta_k_ * (theta - x_ref_k_(10)) * (theta - x_ref_k_(10))
                    + 0.5 * w_v_k_ * (v - x_ref_k_(11)) * (v - x_ref_k_(11))
                    + 0.5 * w_w_k_ * (w - x_ref_k_(12)) * (w - x_ref_k_(12));
    

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
    data->Lx.segment<3>(6) = w_c_k_ * (c - x_ref_k_.segment<3>(6));
    data->Lx(10) = w_theta_k_ * (theta - x_ref_k_(10));
    data->Lx(11) = w_v_k_ * (v - x_ref_k_(11));
    data->Lx(12) = w_w_k_ * (w - x_ref_k_(12));

    // Lxx
    data->Lxx.setZero();
    data->Lxx.block<3,3>(6,6) = w_c_k_ * I3;
    data->Lxx(10,10) = w_theta_k_;
    data->Lxx(11,11) = w_v_k_;
    data->Lxx(12,12) = w_w_k_;

    // Lxu
    data->Lxu.setZero();
    
    // Lu
    data->Lu.setZero();
    data->Lu(0) = w_a_k_ * (a - u_ref_k_(0));
    data->Lu(2) = w_alpha_k_ * (alpha - u_ref_k_(2));
    
    // Luu
    data->Luu.setZero();
    data->Luu(0,0) = w_a_k_;
    data->Luu(2,2) = w_alpha_k_;

    // dynamics 
    data->Fx.setZero();
    data->Fx(6,6) = 1.0;
    data->Fx(7,7) = 1.0;
    data->Fx(10,10) = 1.0;
    data->Fx(11,11) = 1.0;
    data->Fx(12,12) = 1.0;
    data->Fx(10, 12) = dt_;                       // theta depends on w
    data->Fx(6, 10) = -dt_ * v * sin(theta);
    data->Fx(7, 10) =  dt_ * v * cos(theta);
    data->Fx(6, 11) =  dt_ * cos(theta);
    data->Fx(7, 11) =  dt_ * sin(theta);

    data->Fu.setZero();
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


      // z-zmp constraint
      double h_contact = vc_z - 0.0;
      Eigen::MatrixXd Jx_contact = Eigen::MatrixXd::Zero(1, NX_);
      Jx_contact(0,9) = 1.0;

      // Lx
      data->Lx.setZero();
      data->Lx.segment<3>(6) = w_c_k_ * (c - x_ref_k_.segment<3>(6));
      data->Lx(10) = w_theta_k_ * (theta - x_ref_k_(10));
      data->Lx(11) = w_v_k_ * (v - x_ref_k_(11));
      data->Lx(12) = w_w_k_ * (w - x_ref_k_(12));

      // Lxx
      data->Lxx.setZero();
      data->Lxx.block<3,3>(6,6) = w_c_k_ * I3;
      data->Lxx(10,10) = w_theta_k_;
      data->Lxx(11,11) = w_v_k_;
      data->Lxx(12,12) = w_w_k_;

  }





Eigen::VectorXd x_ref_k_;
Eigen::VectorXd u_ref_k_;
private:
double dt_;

// weights
  double w_pcomz_k_;
  double w_pcomxy_k_;
  double w_vcomxy_k_;     
  double w_vcomz_k_; 

  double w_c_k_;
  double w_v_k_;
  double w_theta_k_;
  double w_w_k_;

  double w_a_k_;
  double w_alpha_k_;
  double w_fcz_k_;
  double w_fcxy_k_;
  double w_eq_k_;
  
  const int NX_;
  const int NU_;

  double m_     = 27;
  double d_off_ = 0.1;
  Eigen::Vector3d grav;
  

};

