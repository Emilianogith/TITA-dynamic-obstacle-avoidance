#include "MPC.hpp"



int main(int argc, char** argv) {
    
    labrob::MPC mpc_;

    Eigen::Vector<double, 12> x0 = Eigen::Vector<double, 12>::Zero();


    x0 << 0.002, 0.0002, 0.39,
            0.01, 0.02,  -0.1,
            0.0003, -0.0002, 0.0,
            0.0, 0.0, 0.0; 


        

    double t_msec_ = 0.0;

    for (int i=0; i < 400; i++){

        
        // std::cout << "t_msec_" << t_msec_ << std::endl;
        
        
        // DDP-based MPC 
        if (static_cast<int>(t_msec_) % 1 == 0){
            mpc_.record_logs = true;
            mpc_.t_msec = t_msec_;
        }        
        
        t_msec_ += 1.0;
    
        // std::cout << "x0" << x0 << std::endl;
        mpc_.solve(x0);
        labrob::SolutionMPC sol = mpc_.get_solution();


        x0 << sol.com.pos(0), sol.com.pos(1), sol.com.pos(2),
              sol.com.vel(0), sol.com.vel(1), sol.com.vel(2),
              sol.zmp.pos(0), sol.zmp.pos(1), sol.zmp.pos(2),
              sol.zmp.vel(0), sol.zmp.vel(1), sol.zmp.vel(2);


        }



    return 0;
}