#pragma once

#include <Eigen/Dense>

#include <utils.hpp>
// double wrapToPi(double a) {
//   a = std::fmod(a + M_PI, 2.0 * M_PI);
//   if (a < 0) a += 2.0 * M_PI;
//   return a - M_PI;
// }


namespace labrob {

    class wheel_KF{
        
        static constexpr int NX = 4;
        static constexpr int NZ = 2;
        
        public:
        wheel_KF()
        {
            // compute observation matix
            C << 1, 0, 0, 0,
                 0, 0, 1, 0;

            // ----- P0 -----
            P_k.setZero();
            P_k(0,0)    = sig_q  * sig_q;
            P_k(1,1)    = sig_q_dot  * sig_q_dot;
            P_k(2,2)    = sig_q  * sig_q;
            P_k(3,3)    = sig_q_dot  * sig_q_dot;

            // ----- W -----
            W = (sig_c * sig_c) * Eigen::Matrix2d::Identity();

            x_k.setZero();
        }

        void set_initial_condition(const double& q_wL, const double& q_wR){
            x_k(0) = q_wL;        // q_wL
            x_k(1) = 0.0;         // q_dot_wL
            x_k(2) = q_wR;        // q_wR
            x_k(3) = 0.0;         // q_dot_wR
        }

        Eigen::Vector<double, NX> compute_KF_estimate(const double& Ts, const double& q_wL, const double& q_wR){
            prediction(Ts);
            correction(q_wL, q_wR);
            return x_k;
        }

        private:

        // covariances
        const double sig_q       = 1e-3; 
        const double sig_q_dot   = 0.05; 
        const double sig_q_ddot  = 0.5; 
        const double sig_c       = 1e-3;

        Eigen::Vector<double, NX> x_k;              // x_k is [q_wL, q_dot_wL, q_wR, q_dot_wR]
        Eigen::Matrix<double, NX, NX> P_k;

        Eigen::Matrix<double, NX, NX> A_k;
        Eigen::Matrix<double, NZ, NX> C;

        Eigen::Matrix<double, NX, NX> V;            // process noise
        Eigen::Matrix<double, NZ, NZ> W;            // measurement noise

        void prediction(const double& Ts){

            // compute process model matrix
            Eigen::Matrix2d A_wheel;
            A_wheel << 1, Ts,
                       0, 1;
            
            A_k.setZero();
            A_k.block<2,2>(0,0) = A_wheel;
            A_k.block<2,2>(2,2) = A_wheel;
            
            // ----- V -----
            Eigen::Matrix2d V_block;
            V_block << Ts*Ts*Ts*Ts/4, Ts*Ts*Ts/2,
                       Ts*Ts*Ts/2, Ts*Ts;

            V.setZero();
            V.block<2,2>(0,0) = sig_q_ddot * sig_q_ddot * V_block;
            V.block<2,2>(2,2) = sig_q_ddot * sig_q_ddot * V_block;

            // propagate through the process model
            x_k = A_k * x_k;
            x_k(0) = wrapToPi(x_k(0));              // handle specifically the discontinuoity in +/-pi
            x_k(2) = wrapToPi(x_k(2));              // handle specifically the discontinuoity in +/-pi
            P_k = A_k * P_k * A_k.transpose() + V;
        }

        void correction(const double& q_wL, const double& q_wR){

            Eigen::Vector<double, NZ> z_k;
            z_k(0) = q_wL;
            z_k(1) = q_wR;

            // Kalman gain
            Eigen::Matrix<double, NX, NZ> K_k = P_k * C.transpose() * (C * P_k * C.transpose() + W).inverse();

            const Eigen::Matrix<double, NX, NX> I_KC = Eigen::Matrix<double, NX, NX>::Identity() - K_k * C;
            
            Eigen::Vector<double, NZ> h_pred = C * x_k;
            Eigen::Vector<double, NZ> innovation_k;
            innovation_k(0) = angleError(z_k(0), h_pred(0));
            innovation_k(1) = angleError(z_k(1), h_pred(1));

            // correct the prediction
            x_k = x_k + K_k * innovation_k;         // x_k = x_k + K_k * (z_k - C * x_k);
            x_k(0) = wrapToPi(x_k(0));              // handle specifically the discontinuoity in +/-pi
            x_k(2) = wrapToPi(x_k(2));
            P_k = I_KC * P_k;                       // joseph formula I_KC * P_k * I_KC.transpose() + K_k * W * K_k.transpose()
        }
    };


}       // end namespace labrob