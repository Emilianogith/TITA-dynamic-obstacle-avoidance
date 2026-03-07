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

    struct Odom {
        Eigen::Vector3d position; 
        Eigen::Vector3d velocity;

        Eigen::Vector3d pc_l;  // position of left contact point
        Eigen::Vector3d pc_r;  // position of right contact point

        double w_l;  // angular rolling velocity of left wheel
        double w_r;  // angular rolling velocity of right wheel
    };

    class RobotOdometry{
        
        static constexpr int N_JOINTS = 8;
        
        public:
        RobotOdometry( 
            const pinocchio::Model& robot_model):
        robot_model_(robot_model){
            
            robot_data_ = pinocchio::Data(robot_model_);
            right_leg4_idx_ = robot_model_.getFrameId("right_leg_4");
            left_leg4_idx_ = robot_model_.getFrameId("left_leg_4");
            
            // Initialization
            J_right_wheel_.setZero();
            J_left_wheel_.setZero();
        }

        void reset(
            const Eigen::Vector<double, N_JOINTS>& q_j,
            const Eigen::Quaterniond & q_fb){
            
            Eigen::Vector<double, 3 + 4 + 8> q;
            q << Eigen::Vector3d(0,0,0.0),
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

            robot_base.pc_l = Eigen::Vector3d(left_contact(0), left_contact(1), 0);   
            robot_base.pc_r = Eigen::Vector3d(right_contact(0), right_contact(1), 0);
            

            robot_base.position = Eigen::Vector3d(0,0,(left_contact(2) + right_contact(2)) / 2.0);
            robot_base.velocity.setZero();
            }

        Odom forward_step(const Eigen::Vector<double, N_JOINTS>& q_j,
            const Eigen::Vector<double, N_JOINTS>& q_dot_j,
            const Eigen::Quaterniond & q_fb,
            const Eigen::Vector3d& omega_fb,
            const double& dt){

            Eigen::Vector<double, 3 + 4 + N_JOINTS> q;
            q << Eigen::Vector3d(0,0,0), 
            q_fb.coeffs(),
            q_j;

            Eigen::Vector<double, 3 + 3 + N_JOINTS> qdot;
            qdot << Eigen::Vector3d(0,0,0), 
            omega_fb,
            q_dot_j;
            
            // Compute pinocchio terms
            pinocchio::framesForwardKinematics(robot_model_, robot_data_, q); // update robot_data_.oMf
            pinocchio::computeJointJacobians(robot_model_, robot_data_, q);   // compute joint jacobians

            pinocchio::getFrameJacobian(robot_model_, robot_data_, right_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_wheel_);
            pinocchio::getFrameJacobian(robot_model_, robot_data_, left_leg4_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_wheel_);
            

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

            Eigen::Vector3d omega_l = J_left_wheel_.bottomRows(3) * qdot;
            Eigen::Vector3d omega_r = J_right_wheel_.bottomRows(3) * qdot;

            Eigen::Vector3d omega_virtual_l = (I3 - n_l * n_l.transpose()) * omega_l;
            Eigen::Vector3d omega_virtual_r = (I3 - n_r * n_r.transpose()) * omega_r;

            Eigen::Vector3d pc_l_dot = t_l * n_l.transpose() * omega_l * wheel_radius_;
            Eigen::Vector3d pc_r_dot = t_r * n_r.transpose() * omega_r * wheel_radius_;

            robot_base.pc_l += pc_l_dot * dt;
            robot_base.pc_r += pc_r_dot * dt;
            robot_base.w_l =  n_l.transpose() * omega_l;
            robot_base.w_r =  n_r.transpose() * omega_r;


            Eigen::Vector<double, 3 + N_JOINTS> q_res_dot = qdot.segment<3 + N_JOINTS>(3);

            Eigen::Matrix3d R_base = q_fb.toRotationMatrix();

            Eigen::Vector3d v_from_left = R_base * J_left_wheel_.block(0,0,3,3).inverse() * (pc_l_dot + omega_virtual_l.cross(robot_base.pc_l) * wheel_radius_ - J_left_wheel_.block(0,3,3 + N_JOINTS,3 + N_JOINTS) * q_res_dot);
            Eigen::Vector3d v_from_right = R_base * J_right_wheel_.block(0,0,3,3).inverse() * (pc_r_dot + omega_virtual_r.cross(robot_base.pc_r) * wheel_radius_ - J_right_wheel_.block(0,3,3 + N_JOINTS,3 + N_JOINTS) * q_res_dot);


            robot_base.velocity = (v_from_left + v_from_right) / 2.0;


            const auto& r_wheel_center = robot_data_.oMf[right_leg4_idx_];
            const auto& l_wheel_center = robot_data_.oMf[left_leg4_idx_];
            Eigen::Vector3d right_rCP = labrob::get_rCP(r_wheel_center.rotation(), wheel_radius_);
            Eigen::Vector3d left_rCP = labrob::get_rCP(l_wheel_center.rotation(), wheel_radius_);
            Eigen::Vector3d right_contact = r_wheel_center.translation() + right_rCP;
            Eigen::Vector3d left_contact = l_wheel_center.translation() + left_rCP;
            

            Eigen::Vector3d p_from_left = robot_base.pc_l - left_contact;
            Eigen::Vector3d p_from_right = robot_base.pc_r - right_contact;

            robot_base.position = (p_from_left + p_from_right) / 2.0;



            return robot_base;

        }


        private:    

        Odom robot_base;

        pinocchio::Model robot_model_;
        pinocchio::Data robot_data_;

        double wheel_radius_ = 0.0925;
        double g = 9.81;

        pinocchio::FrameIndex right_leg4_idx_;
        pinocchio::FrameIndex left_leg4_idx_;

        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6 + 8> J_right_wheel_;
        Eigen::Matrix<double, 6, 6 + 8> J_left_wheel_;


    };

}       // end namespace labrob