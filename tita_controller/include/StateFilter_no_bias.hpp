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
        
        static constexpr int NX = 12;
        static constexpr int NU = 14;
        static constexpr int NP = 12;
        static constexpr int NZ = 3 * 2 + 2;
        static constexpr int N_JOINTS = 8;
        
        public:
        KF(const pinocchio::Model& robot_model):
        robot_model_(robot_model){
            
            robot_data_ = pinocchio::Data(robot_model_);
            right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
            left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
            
            J_right_wheel_.setZero();
            J_left_wheel_.setZero();

            // Standard deviations for P0
            const double sig_p  = 0.05; 
            const double sig_v  = 0.01; 
            const double sig_c  = 0.01;


            const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

            // ----- P0 -----
            P_k.setZero();
            P_k.block<3,3>(0,0)    = (sig_p  * sig_p ) * I3;
            P_k.block<3,3>(3,3)    = (sig_v  * sig_v ) * I3;
            P_k.block<3,3>(6,6)    = (sig_c  * sig_c ) * I3;
            P_k.block<3,3>(9,9)    = (sig_c  * sig_c ) * I3;

            compute_noise_process_matrix();

            // ----- W -----
            W.setZero();
            W.block<3,3>(0,0) = (sig_z * sig_z) * I3;
            W.block<3,3>(3,3) = (sig_z * sig_z) * I3; 

            // only if in contact
            W(6,6)  = sig_z_contact * sig_z_contact;
            W(7,7) = sig_z_contact * sig_z_contact;


            x_k.setZero();
        }


        void set_initial_condition(const Eigen::Vector<double, N_JOINTS>& q_j,
            const Eigen::Quaterniond & q_fb){
            
            Eigen::Vector<double, 3 + 4 + N_JOINTS> q;
            q << Eigen::Vector3d(0,0,0),
            q_fb.coeffs(),
            q_j;
            
            // Compute pinocchio terms
            pinocchio::framesForwardKinematics(robot_model_, robot_data_, q); // update robot_data_.oMf            

            const auto& r_wheel_center = robot_data_.oMf[right_leg4_idx_];
            const auto& l_wheel_center = robot_data_.oMf[left_leg4_idx_];
            Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center.rotation(), wheel_radius_);
            Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center.rotation(), wheel_radius_);
            Eigen::Vector3d right_contact = r_wheel_center.translation() + right_rCP;
            Eigen::Vector3d left_contact = l_wheel_center.translation() + left_rCP;

            x_k.segment<3>(0) = Eigen::Vector3d(0,0,(left_contact(2) + right_contact(2)) / 2.0);        // p_fb
            x_k.segment<3>(3).setZero();                                                                // v_fb
            x_k.segment<3>(6) = Eigen::Vector3d(left_contact(0), left_contact(1), 0);                   // p_cL
            x_k.segment<3>(9) = Eigen::Vector3d(right_contact(0), right_contact(1), 0);                 // p_cR
        }



        Eigen::Vector<double, NX> compute_KF_estimate(const Eigen::Vector<double, NU>& u_k, const Eigen::Vector<double, NP>& params, const double& dt, bool in_contact){
            
            Eigen::Vector<double, N_JOINTS> q_joint = params.tail<N_JOINTS>();

            q_base.coeffs() << params(0), params(1), params(2), params(3);  // (x,y,z,w)
            q_base.normalize();

            Eigen::Vector<double, 3 + 4 + N_JOINTS> q;
            q << Eigen::Vector3d(0,0,0), 
            q_base.coeffs(),
            q_joint;
            
            // Compute pinocchio terms
            pinocchio::framesForwardKinematics(robot_model_, robot_data_, q); // update robot_data_.oMf
            pinocchio::computeJointJacobians(robot_model_, robot_data_, q);   // compute joint jacobians

            pinocchio::getFrameJacobian(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_);
            pinocchio::getFrameJacobian(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_);
            
            // if robot is flying increase contact noise
            if(!in_contact){
                sig_c_proc = 1.0;
                compute_noise_process_matrix();
            } else{
                sig_p_proc  = 0.05;     // 0.05
                sig_v_proc  = 0.2;      // 0.2
                sig_c_proc  = 1e-4;  
                compute_noise_process_matrix();
            }

            prediction(u_k, dt);
            correction();
            return x_k;
        }



        private:

        // Discrete-time process noise 
        double sig_p_proc  = 0.05;  
        double sig_v_proc  = 0.001;  
        double sig_c_proc  = 1e-4;  

        // Measurement noise for W 
        double sig_z = 2e-5;  
        double sig_z_contact = 1e-15;


        Eigen::Vector<double, NX> x_k;      // x_k is [p_fb, v_fb, pc_L, pc_R]  p_fb and v_fb are expressed in world frame
        Eigen::Matrix<double, NX, NX> P_k;

        Eigen::Matrix<double, NX, NX> V;            // process noise
        Eigen::Matrix<double, NZ, NZ> W;            // measurement noise

        Eigen::Quaterniond q_base;

        pinocchio::Model robot_model_;
        pinocchio::Data robot_data_;

        double wheel_radius_ = 0.0925;
        double g = 9.744;

        pinocchio::FrameIndex right_leg4_idx_;
        pinocchio::FrameIndex left_leg4_idx_;

        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6 + N_JOINTS> J_right_wheel_;
        Eigen::Matrix<double, 6, 6 + N_JOINTS> J_left_wheel_;

        void compute_noise_process_matrix(){
            const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
            
            // ----- V -----
            V.setZero();
            V.block<3,3>(0,0)    = (sig_p_proc  * sig_p_proc ) * I3;
            V.block<3,3>(3,3)    = (sig_v_proc  * sig_v_proc ) * I3;
            V.block<3,3>(6,6)    = (sig_c_proc  * sig_c_proc ) * I3;
            V.block<3,3>(9,9)    = (sig_c_proc  * sig_c_proc ) * I3;
        }

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


            Eigen::Matrix<double, 3, 3 + N_JOINTS> H_l = t_l * n_l.transpose() *  J_left_wheel_.block<3,3 + N_JOINTS>(3,3) * wheel_radius_;
            Eigen::Matrix<double, 3, 3 + N_JOINTS> H_r = t_r * n_r.transpose() *  J_right_wheel_.block<3,3 + N_JOINTS>(3,3) * wheel_radius_;


            Eigen::Matrix<double, NX, NX> A_k = Eigen::Matrix<double, NX, NX>::Zero();
            A_k.block<3,3>(0, 3) = I3;
            A_k = Eigen::Matrix<double, NX, NX>::Identity() + dt * A_k;

            Eigen::Matrix<double, NX, NU> B_k = Eigen::Matrix<double, NX, NU>::Zero();
            B_k.block<3,3>(3, 0) = R_base;
            B_k.block<3,3 + N_JOINTS>(6, 3) = H_l;
            B_k.block<3,3 + N_JOINTS>(9, 3) = H_r;
            B_k = B_k * dt;

            Eigen::Vector<double, NX> c = Eigen::Vector<double, NX>::Zero();
            c(5) = -g;
            // Eigen::Vector3d bias_acc = Eigen::Vector3d(0,0,-0.07);
            // c.segment<3>(3) = -R_base * bias_acc + Eigen::Vector3d(0,0,-g);

            c = c * dt;

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
            z_k.segment<2>(6).setZero();


            Eigen::Matrix<double, NZ, NX> C_k = Eigen::Matrix<double, NZ, NX>::Zero();
            C_k.block<3,3>(0,0) = -I3;
            C_k.block<3,3>(3,0) = -I3;
            C_k.block<3,3>(0,6) = I3;
            C_k.block<3,3>(3,9) = I3;
            C_k(6,8) = 1.0;
            C_k(7,11) = 1.0;

            // Kalman gain
            Eigen::Matrix<double, NX, NZ> K_k = P_k * C_k.transpose() * (C_k * P_k * C_k.transpose() + W).inverse();

            const Eigen::Matrix<double, NX, NX> I_KC = Eigen::Matrix<double, NX, NX>::Identity() - K_k * C_k;
            
            // correct the prediction
            x_k = x_k + K_k * (z_k - C_k * x_k);
            P_k = I_KC * P_k;               // joseph formula I_KC * P_k * I_KC.transpose() + K_k * W * K_k.transpose()
        }
    };


}       // end namespace labrob