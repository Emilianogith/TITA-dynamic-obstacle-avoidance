#include "MPC.hpp"
#include "walkingPlanner.hpp"
#include "Logger.hpp"

#include <chrono>


int main(int argc, char** argv) {
    

    labrob::walkingPlanner walkingPlanner_;
    labrob::MPC mpc_;
    labrob::Logger logger_;

    double controller_timestep_msec_ = 2.0;

    Eigen::Vector3d p_CoM = Eigen::Vector3d(0.002, 0.001, 0.39);

    Eigen::VectorXd x0(22);
    x0 << p_CoM,
      0.0, 0.0, 0.0,            // vcom
      0.0, 0.0, 0.0,            // rpy
      0.0, 0.0, 0.0,            // ang_vel
      0.0, -0.5, 0.0,           // pr
      0.0,                      // vr
      0.0,                      // vr_z
      0.0, 0.5, 0.0,            // pl
      0.0,                      // vl
      0.0;                      // vl_z

    // plan the offline trajectory
    walkingPlanner_.offline_plan(0.001 * controller_timestep_msec_, true, p_CoM);
    mpc_.set_planner(walkingPlanner_, 0.001 * controller_timestep_msec_);
    mpc_.init_solver(x0);
    
    
    logger_.reserve(20000);
        

    double t_msec_ = 0.0;

    auto t_start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < 400; i++){

        auto t_in = std::chrono::high_resolution_clock::now();
        
        // std::cout << "t_msec_" << t_msec_ << std::endl;
     
     
        mpc_.set_current_t_msec(t_msec_);
            
        
        
        
        // std::cout << "x0" << x0 << std::endl;
        mpc_.solve(x0);
        labrob::SolutionMPC sol = mpc_.get_solution();



        auto t_out = std::chrono::high_resolution_clock::now();

        double time_mpc = std::chrono::duration<double, std::micro>(t_out - t_in).count();
        std::cout << "time_mpc us " << time_mpc << std::endl;

        // log data
        labrob::MPCEntry mpc_entry;
        mpc_entry.time_ms = t_msec_;

        mpc_entry.X = mpc_.X;
        mpc_entry.U = mpc_.U;
    
        logger_.log_mpc_data(std::move(mpc_entry));

        
        labrob::TimingEntry timing;
        timing.time_mpc_us = time_mpc;
        timing.time_wbc_us = 0.0;
        timing.total_time_us = 0.0;

        logger_.log_timing_data(std::move(timing));

        Eigen::Matrix2d R_yaw;
        const double& yaw = sol.rpy.pos(2);
        R_yaw << std::cos(yaw), - std::sin(yaw),
                 std::sin(yaw), std::cos(yaw);
        
        double vr = R_yaw.transpose().row(0).dot(sol.pr.vel.segment<2>(0));
        double vl = R_yaw.transpose().row(0).dot(sol.pl.vel.segment<2>(0));



        
        x0.segment<3>(0)  = sol.com.pos;
        x0.segment<3>(3)  = sol.com.vel; 
        x0.segment<3>(6)  = sol.rpy.pos;
        x0.segment<3>(9)  = sol.rpy.vel;
        x0.segment<3>(12) = sol.pr.pos;
        x0(15)            = vr;
        x0(16)            = sol.pr.vel(2);
        x0.segment<3>(17) = sol.pl.pos;
        x0(20)            = vl;
        x0(21)            = sol.pl.vel(2);


        // disturbance
        if (i==100){
            // x0(1) +=0.01;
            x0(6)  = 0.1;  // disturbo pitch
            x0(7)  = 0.2;  // disturbo pitch
            x0(8)  = 0.3;  // disturbo pitch
        }
    

        t_msec_ += 2.0;
        
    }
    
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
    std::cout << "Complete program time: " << duration.count() << " ms" << std::endl;


    logger_.save_log_data();
    

    return 0;
}