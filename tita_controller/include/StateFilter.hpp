#pragma once

#include <Eigen/Dense>

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

#include <pinocchio/algorithm/jacobian.hpp>

#include "utils.hpp"

namespace labrob {

    class KF{
        
        static constexpr int NX = 18;
        static constexpr int NU = 14;
        static constexpr int NP = 12;
        static constexpr int NZ = 3 * 2;
        
        public:
        KF(const Eigen::Vector<double, NX>& x0, const pinocchio::Model& robot_model):
        robot_model_(robot_model){
            
            robot_data_ = pinocchio::Data(robot_model_);
            right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
            left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
            
            J_right_wheel_.setZero();
            J_left_wheel_.setZero();
            
            // Initialization
            x_k = x0;

            // Standard deviations for P0
            const double sig_p  = 0.2; 
            const double sig_v  = 0.05; 
            const double sig_c  = 0.01;
            const double sig_ba = 1e-3; 
            const double sig_bw = 1e-3;     // 0.05

            // Discrete-time process noise 
            const double sig_p_proc  = 0.05;          // 5e-3
            const double sig_v_proc  = 0.03;          // 0.5
            const double sig_c_proc  = 1e-6;         // 1e-4
            const double sig_ba_rw   = 1e-5;         // 5e-4
            const double sig_bw_rw   = 1e-5;         // 5e-5

            // Measurement noise for W 
            const double sig_z = 0.001;               // 0.02

            const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

            // ----- P0 -----
            P_k.setZero();
            P_k.block<3,3>(0,0)    = (sig_p  * sig_p ) * I3;
            P_k.block<3,3>(3,3)    = (sig_v  * sig_v ) * I3;
            P_k.block<3,3>(6,6)    = (sig_c  * sig_c ) * I3;
            P_k.block<3,3>(9,9)    = (sig_c  * sig_c ) * I3;
            P_k.block<3,3>(12,12)  = (sig_ba * sig_ba) * I3;
            P_k.block<3,3>(15,15)  = (sig_bw * sig_bw) * I3;

            // ----- V -----
            V.setZero();
            V.block<3,3>(0,0)    = (sig_p_proc  * sig_p_proc ) * I3;
            V.block<3,3>(3,3)    = (sig_v_proc  * sig_v_proc ) * I3;
            V.block<3,3>(6,6)    = (sig_c_proc  * sig_c_proc ) * I3;
            V.block<3,3>(9,9)    = (sig_c_proc  * sig_c_proc ) * I3;
            V.block<3,3>(12,12)  = (sig_ba_rw   * sig_ba_rw  ) * I3;
            V.block<3,3>(15,15)  = (sig_bw_rw   * sig_bw_rw  ) * I3;

            // ----- W -----
            W.setZero();
            W.block<3,3>(0,0) = (sig_z * sig_z) * I3;
            W.block<3,3>(3,3) = (sig_z * sig_z) * I3; 
        }



        Eigen::Vector<double, NX> compute_KF_estimate(const Eigen::Vector<double, NU>& u_k, const Eigen::Vector<double, NP>& params, const double& dt){
            
            Eigen::Vector<double, 8> q_joint = params.tail<8>();

            q_base.coeffs() << params(0), params(1), params(2), params(3);  // (x,y,z,w)
            q_base.normalize();

            Eigen::Vector<double, 3 + 4 + 8> q;
            q << Eigen::Vector3d(0,0,0), 
            q_base.coeffs(),
            q_joint;
            
            // Compute pinocchio terms
            pinocchio::framesForwardKinematics(robot_model_, robot_data_, q); // update robot_data_.oMf
            pinocchio::computeJointJacobians(robot_model_, robot_data_, q);   // compute joint jacobians

            pinocchio::getFrameJacobian(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_);
            pinocchio::getFrameJacobian(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_);
            
            prediction(u_k, dt);
            correction();
            return x_k;
        }



        private:

        Eigen::Vector<double, NX> x_k;      // x_k is [p_fb, v_fb, pc_L, pc_R]  p_fb and v_fb are expressed in world frame
        Eigen::Matrix<double, NX, NX> P_k;

        Eigen::Matrix<double, NX, NX> V;            // process noise
        Eigen::Matrix<double, NZ, NZ> W;            // measurement noise

        Eigen::Quaterniond q_base;

        pinocchio::Model robot_model_;
        pinocchio::Data robot_data_;

        double wheel_radius_ = 0.0925;
        double g = 9.81;

        pinocchio::FrameIndex right_leg4_idx_;
        pinocchio::FrameIndex left_leg4_idx_;

        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6 + 8> J_right_wheel_;
        Eigen::Matrix<double, 6, 6 + 8> J_left_wheel_;

        void prediction(const Eigen::Vector<double, NU>& u_k, const double& dt){
            
            Eigen::Matrix3d R_base = q_base.toRotationMatrix();

            // t_l, n_l , J_ang with pinocchio
            // compute with differential kinematics
            Eigen::Matrix3d r_wheel_R = robot_data_.oMf[right_leg4_idx_].rotation();
            Eigen::Matrix3d l_wheel_R = robot_data_.oMf[left_leg4_idx_].rotation();
            Eigen::Matrix3d r_virtual_frame_R = labrob::compute_virtual_frame(r_wheel_R);
            Eigen::Matrix3d l_virtual_frame_R = labrob::compute_virtual_frame(l_wheel_R);

            Eigen::Vector3d t_l = l_virtual_frame_R.col(0);
            Eigen::Vector3d t_r = r_virtual_frame_R.col(0);
            Eigen::Vector3d n_l = l_virtual_frame_R.col(1);
            Eigen::Vector3d n_r = r_virtual_frame_R.col(1);


            Eigen::Matrix<double, 3, 6 + 8> H_l = t_l * n_l.transpose() *  J_left_wheel_.bottomRows<3>() * wheel_radius_;
            Eigen::Matrix<double, 3, 6 + 8> H_r = t_r * n_r.transpose() *  J_right_wheel_.bottomRows<3>() * wheel_radius_;
            H_l.block<3, 3>(0, 0) *= R_base.transpose();        // rotates v_fb input in base frame for pinocchio conventions
            H_r.block<3, 3>(0, 0) *= R_base.transpose(); 

            Eigen::Matrix<double, 3, 3> H_lv = H_l.block<3,3>(0,0);
            Eigen::Matrix<double, 3, 3> H_lw = H_l.block<3,3>(0,3);
            Eigen::Matrix<double, 3, 8> H_lJ = H_l.block<3, 8>(0,6);

            Eigen::Matrix<double, 3, 3> H_rv = H_r.block<3,3>(0,0);
            Eigen::Matrix<double, 3, 3> H_rw = H_r.block<3,3>(0,3);
            Eigen::Matrix<double, 3, 8> H_rJ = H_r.block<3, 8>(0,6);


            Eigen::Matrix<double, NX, NX> A_k = Eigen::Matrix<double, NX, NX>::Identity();
            A_k.block<3,3>(0, 3) = I3 * dt;
            A_k.block<3,3>(0, 12) = - R_base * dt * dt * 0.5;
            A_k.block<3,3>(3, 12) = - R_base * dt;
            A_k.block<3,3>(6, 3) = H_lv * dt;
            A_k.block<3,3>(9, 3) = H_rv * dt;
            A_k.block<3,3>(6, 15) = - H_lw * dt;
            A_k.block<3,3>(9, 15) = - H_rw * dt;

            Eigen::Matrix<double, NX, NU> B_k = Eigen::Matrix<double, NX, NU>::Zero();
            B_k.block<3,3>(0, 0) = R_base * dt * dt * 0.5;
            B_k.block<3,3>(3, 0) = R_base * dt;
            B_k.block<3,3>(6, 3) = H_lw * dt;
            B_k.block<3,3>(9, 3) = H_rw * dt;
            B_k.block<3,8>(6, 6) = H_lJ * dt;
            B_k.block<3,8>(9, 6) = H_rJ * dt;

            Eigen::Vector<double, NX> c = Eigen::Vector<double, NX>::Zero();
            c(2) = -g * dt * dt * 0.5;
            c(5) = -g * dt;

            // propagate through the process model
            x_k = A_k * x_k + B_k * u_k + c;
            P_k = A_k * P_k * A_k.transpose() + V;
        }

        void correction(){

            const auto& r_wheel_center = robot_data_.oMf[right_leg4_idx_];
            const auto& l_wheel_center = robot_data_.oMf[left_leg4_idx_];
            Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center.rotation(), wheel_radius_);
            Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center.rotation(), wheel_radius_);
            Eigen::Vector3d right_contact = r_wheel_center.translation() + right_rCP;
            Eigen::Vector3d left_contact = l_wheel_center.translation() + left_rCP;
            
            Eigen::Vector<double, NZ> z_k;
            z_k.segment<3>(0) = left_contact;
            z_k.segment<3>(3) = right_contact;


            Eigen::Matrix<double, NZ, NX> C_k = Eigen::Matrix<double, NZ, NX>::Zero();
            C_k.block<3,3>(0,0) = -I3;
            C_k.block<3,3>(3,0) = -I3;
            C_k.block<3,3>(0,6) = I3;
            C_k.block<3,3>(3,9) = I3;

            // Kalman gain
            Eigen::Matrix<double, NX, NZ> K_k = P_k * C_k.transpose() * (C_k * P_k * C_k.transpose() + W).inverse();

            const Eigen::Matrix<double, NX, NX> I_KC = Eigen::Matrix<double, NX, NX>::Identity() - K_k * C_k;
            
            // correct the prediction
            x_k = x_k + K_k * (z_k - C_k * x_k);
            P_k = I_KC * P_k * I_KC.transpose() + K_k * W * K_k.transpose();    // joseph formula I_KC * P_k * I_KC.transpose() + K_k * W * K_k.transpose()
            // P_k = I_KC * P_k;            
        }
    };


}       // end namespace labrob