#include "MPC.hpp"

#include <chrono>


int main(int argc, char** argv) {
    
    
    Eigen::Vector<double, 10> x0 = Eigen::Vector<double, 10>::Zero();
    x0 << 0.002, 0.0003, 0.39,
    0.01, -0.02,  -0.1,
    0.0003, -0.0002,
    0.0, 0.0; 
    
    
    labrob::MPC mpc_(x0);
        

    double t_msec_ = 0.0;

    auto t_start = std::chrono::high_resolution_clock::now();
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

        // double theta = sol.theta;

        x0 << sol.com.pos(0), sol.com.pos(1), sol.com.pos(2),
              sol.com.vel(0), sol.com.vel(1), sol.com.vel(2),
              sol.zmp.pos(0), sol.zmp.pos(1),
              sol.zmp.vel(0), sol.zmp.vel(1);


        }

        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
        std::cout << "Complete program time: " << duration.count() << " ms" << std::endl;



    return 0;
}