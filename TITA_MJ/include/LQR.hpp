#pragma once
#include <Eigen/Dense>

namespace labrob {

struct SolutionLQR { 

  struct Com {
    double pos; 
    double vel; 
    double acc;
  };

  struct Zmp {
    double pos; 
    double vel; 
    double acc;
  };

  Com com;
  Zmp zmp;
};

class LQR{
    public:

    LQR(double z0):z0(z0){

        double eta = std::sqrt(g / z0);

        P.resize(N+1);
        K.resize(N);

        // Exact discretization
        const double s = std::sinh(eta*Ts), c = std::cosh(eta*Ts);
        A << c, s/eta,
            eta*s, c;
        
        B << 1 - c,
            -eta*s;


        Q(0,0) = 2250.0;           // pos weight                         
        Q(1,1) = 2900.0;           // vel weight   

        R(0,0) = 2920;       // input (ZMP) weight 
        
        Qf(0,0) = 2250.0;             // pos weight
        Qf(1,1) = 2900.0;             // vel weight
    }


    void solve(const double x_com, const double vx_com, const double ax_com,
               const double x_prev_zmp, const double vx_prev_zmp, const double ax_prev_zmp){

        // Terminal condition
        P[N] = Qf;

        // Backward Riccati (time-varying LQR)
        for (int k = N-1; k >= 0; --k) {
            // S = R + B^T P_{k+1} B  (scalar here)
            double S = (R + (B.transpose() * P[k+1] * B))(0,0);
            // K_k = S^{-1} B^T P_{k+1} A   (1x2)
            K[k] = (1.0 / S) * (B.transpose() * P[k+1] * A);
            // P_k = Q + A^T (P_{k+1} - P_{k+1} B S^{-1} B^T P_{k+1}) A
            Eigen::Matrix2d term = P[k+1] - (P[k+1] * B) * (1.0 / S) * (B.transpose() * P[k+1]);
            P[k] = Q + A.transpose() * term * A;
        }

        Eigen::Vector2d x_ref(c_star, 0.0);

        // --- Closed-loop rollout over the horizon (predict)
        x_ZMP_des  = Eigen::VectorXd::Zero(N);
        v_ZMP_des  = Eigen::VectorXd::Zero(N);
        a_ZMP_des  = Eigen::VectorXd::Zero(N);
        x_CoM_des = Eigen::VectorXd::Zero(N+1);
        v_CoM_des = Eigen::VectorXd::Zero(N+1);
        a_CoM_des  = Eigen::VectorXd::Zero(N+1);

        // Current state (measurements)
        double c_now = x_com;
        double cdot_now = vx_com;
        Eigen::Vector2d x(c_now, cdot_now);

        x_CoM_des(0) = x(0);
        v_CoM_des(0) = x(1);
        a_CoM_des(0) = ax_com;

        double u_prev = x_prev_zmp;
        double v_prev = vx_prev_zmp;
        double a_prev = ax_prev_zmp;

        for (int k = 0; k < N; ++k) {
            Eigen::Vector2d e = x - x_ref;
            double u = u_ref - (K[k] * e)(0,0);     // optimal input at stage k
            // apply / predict
            x = A * x + B * u;
            x_ZMP_des(k) = u;
            
            x_CoM_des(k+1) = x(0);
            v_CoM_des(k+1) = x(1);
            a_CoM_des(k+1) = eta*eta* (x(0) - u);

            // vel, acc derivation
            const double vmax=1.0, amax=3.0, jmax=20.0; // tune
            double u_rate = std::clamp((u - u_prev)/Ts, -vmax, vmax);
            double u_lim  = u_prev + u_rate*Ts;

            double v_cmd  = (u_lim - u_prev)/Ts;
            double dv     = std::clamp(v_cmd - v_prev, -amax*Ts, amax*Ts);
            double v_lim  = v_prev + dv;

            double a_cmd  = (v_lim - v_prev)/Ts;
            double da     = std::clamp(a_cmd - a_prev, -jmax*Ts, jmax*Ts);
            double a_lim  = a_prev + da;

            x_ZMP_des(k) = u_lim;  v_ZMP_des(k) = v_lim;  a_ZMP_des(k) = a_lim;
            u_prev = u_lim;  v_prev = v_lim;  a_prev = a_lim;
        }  
    }

    SolutionLQR get_solution() const {
        return {
        {x_CoM_des(1), v_CoM_des(1), a_CoM_des(1)},   // COM
        {x_ZMP_des(1), v_ZMP_des(1), a_ZMP_des(1)}    // ZMP
        };
    }

    void record_logs(double t_msec_){
        mpc_com_log_file_.open("/tmp/mpc_com.txt");
        mpc_zmp_log_file_.open("/tmp/mpc_zmp.txt");
        mpc_com_log_file_ << "t_msec_: " << t_msec_ << std::endl << x_CoM_des.transpose() << std::endl;
        mpc_zmp_log_file_ << "t_msec_: " << t_msec_ << std::endl << x_ZMP_des.transpose() << std::endl;
    }

    private:
        double g = 9.81;
        double z0;                                  // CoM height [m]
        double eta;                                 // sqrt(g/z0)
        double Ts = 0.01;                           // sampling time [s]
        int N = 100;                                // horizon length


        // --- Reference tracking (constant setpoint c*)
        double c_star = 0.0;
        double u_ref = c_star;   // steady-state ZMP = c*

        Eigen::Matrix2d A;
        Eigen::Vector2d B;


        // --- LQR weights
        Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();   
        Eigen::Matrix<double,1,1> R;               
        Eigen::Matrix2d Qf = Eigen::Matrix2d::Zero();   


        // --- Storage for TV-LQR
        std::vector<Eigen::Matrix2d> P;
        std::vector<Eigen::RowVector2d> K;

        // --- Closed-loop rollout over the horizon (predict)
        Eigen::VectorXd x_ZMP_des  = Eigen::VectorXd::Zero(N);
        Eigen::VectorXd v_ZMP_des  = Eigen::VectorXd::Zero(N);
        Eigen::VectorXd a_ZMP_des  = Eigen::VectorXd::Zero(N);
        Eigen::VectorXd x_CoM_des = Eigen::VectorXd::Zero(N+1);
        Eigen::VectorXd v_CoM_des = Eigen::VectorXd::Zero(N+1);
        Eigen::VectorXd a_CoM_des  = Eigen::VectorXd::Zero(N+1);

        //logs
        std::ofstream mpc_com_log_file_;
        std::ofstream mpc_zmp_log_file_;
};

} // end namespace labrob