#include "MPC.hpp"
#include "walkingPlanner.hpp"

#include <chrono>


int main(int argc, char** argv) {
    

    labrob::walkingPlanner walkingPlanner_;
    labrob::MPC mpc_;

    Eigen::VectorXd x_IN(18);
    double ang = 0.0;   // M_PI + M_PI/6;
    double theta = ang + M_PI/2;   // M_PI + M_PI/6;
    x_IN << 0.002, 0.001, 0.39,       // pcom
          -0.03, 0.003, 0.0,         // vcom
          cos(theta), sin(theta), 0.0,              // pl
          -cos(theta), -sin(theta), 0.0,             // pr
          0.0, 0.0, 0.0,              // dpl
          0.0, 0.0, 0.0;              // dpr
    mpc_.set_planner(walkingPlanner_, 0.001 * 2);
    mpc_.init_solver(x_IN);
    mpc_.solve(x_IN);
    
    
        

    double t_msec_ = 0.0;

    auto t_start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < 400; i++){

        
        // std::cout << "t_msec_" << t_msec_ << std::endl;
        
        
        // DDP-based MPC 
        if (static_cast<int>(t_msec_) % 1 == 0){
            mpc_.record_logs = true;
            mpc_.t_msec = t_msec_;
        }        
        
        
        
        // std::cout << "x_IN" << x_IN << std::endl;
        mpc_.solve(x_IN);
        labrob::SolutionMPC sol = mpc_.get_solution();
        
        
        x_IN.segment<3>(0) = sol.com.pos;
        x_IN.segment<3>(3) = sol.com.vel;
        x_IN.segment<3>(6) = sol.pl.pos;
        x_IN.segment<3>(9) = sol.pr.pos;
        x_IN.segment<3>(12) = sol.pl.vel;
        x_IN.segment<3>(15) = sol.pr.vel;
        
        t_msec_ += 2.0;
        
    }
    
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
    std::cout << "Complete program time: " << duration.count() << " ms" << std::endl;
    
    

    return 0;
}