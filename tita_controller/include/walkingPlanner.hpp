#pragma once

#include <iostream>
#include <fstream>

#include <Eigen/Dense>



namespace labrob {

class walkingPlanner {
  private:
    static constexpr int NX = 13; 
    static constexpr int NU = 9; 
    static constexpr int T = 7;              // in sec
    
    double dt_;
    int N_STEP_;     //n. of timesteps
    
    double grav = 9.81;
    double m = 27.68978;
    
    // ref trajectory
    Eigen::MatrixXd x_ref;
    Eigen::MatrixXd u_ref;

    bool log_plan_ = false;
    
  public:  
  walkingPlanner(){};


  void offline_plan(Eigen::Vector3d& p0, const double& dt, bool log_plan){

    dt_ = dt;
    N_STEP_ = static_cast<int>(T / dt_);     //n. of timesteps
    log_plan_ = log_plan;

    x_ref.setZero(NX, N_STEP_);
    u_ref.setZero(NU, N_STEP_-1);

    // fast speed trajectory
    // double T1 = 1;
    // double T2 = 2;
    // double T3 = 3;
    // double T4 = 4;
    // double T5 = 5;
    // double T6 = 7;
    // double T7 = 9;


    
    // constant velocity profile
    double vz          = 0.0;
    double v_contact_z = 0.0;
    double v           = 0.0;
    double omega       = 0.0;
    double theta0      = 0.0;

    double x0          = p0(0);
    double y0          = p0(1);
    double z0          = p0(2);
    double z0_contact  = 0.0;

    double z_min       = 0.25;
    double z_max       = 0.42;
 
    double x = x0;
    double y = y0;
    double z = z0;

    for (int t_step = 0; t_step < N_STEP_; ++t_step){
        double t = t_step * dt_;


        // if (t < T1){
        //   v = 1.1;
        // } else if (t >= T1  && t < T2){
        //   v = 2.5;
        // } else if (t >= T2 && t < T3){
        //   v = 4.0;
        // } else if (t >= T3  && t < T4){
        //   v = 5.0;
        // } else if (t >= T4 && t < T5){
        //   v = 4.0;
        // } else if (t >= T5 && t < T6){
        //   v = 3.0;
        // } else if (t >= T6 && t < T7){
        //   v = 2.0;
        // } else if (t >= T7){
        //   v = 1.0;
        // }


        // unicycle equations
        double theta = theta0 + omega * t;
        double vx = v * cos(theta);
        double vy = v * sin(theta);

        // Euler integration
        if (abs(omega) > 1e-9) {      // assume v always constant along the trajectory and integrate from p0
            x = x0 + (v / omega) * (sin(theta0 + omega * t) - sin(theta0));
            y = x0 - (v / omega) * (cos(theta0 + omega * t) - cos(theta0));
        } else {                      // assume v piece-wise constant along the trajectory and compute the increment from previous position
            x  += v * cos(theta0) * dt_;
            y  += v * sin(theta0) * dt_;
        }

        z = std::clamp(z + vz * dt_, z_min, z_max);
        double z_contact = z0_contact + v_contact_z * t;

        if (z <= z_min || z >= z_max){
          vz = 0.0;
        }

        // stop at last state
        if (t_step == N_STEP_ - 1){
          vx = 0.0; vy = 0.0; vz = 0.0;
          v_contact_z = 0.0; v = 0.0; omega = 0.0;
        }

        x_ref.col(t_step)(0)  = x;
        x_ref.col(t_step)(1)  = y;
        x_ref.col(t_step)(2)  = z;

        x_ref.col(t_step)(3)  = vx;
        x_ref.col(t_step)(4)  = vy;
        x_ref.col(t_step)(5)  = vz;

        x_ref.col(t_step)(6)  = x;
        x_ref.col(t_step)(7)  = y;
        x_ref.col(t_step)(8)  = z_contact;

        x_ref.col(t_step)(9)  = v_contact_z;
        x_ref.col(t_step)(10) = theta;
        x_ref.col(t_step)(11) = v;
        x_ref.col(t_step)(12) = omega;  
    }

   

    for (int t_step = 0; t_step < N_STEP_ - 1; ++t_step){

        u_ref.col(t_step)(0) = 0.0;     // a
        u_ref.col(t_step)(1) = 0.0;     // ac_z
        u_ref.col(t_step)(2) = 0.0;     // alpha
        
        u_ref.col(t_step)(3) = 0;         // fl_x
        u_ref.col(t_step)(4) = 0;         // fl_y
        u_ref.col(t_step)(5) = m*grav/2;  // fl_z

        u_ref.col(t_step)(6) = 0;         // fr_x
        u_ref.col(t_step)(7) = 0;         // fr_y
        u_ref.col(t_step)(8) = m*grav/2;  // fr_z
    }



    if (log_plan_){
      // create folder if it does not exist
      std::string folder = "/tmp/plan/" ;
      std::string command = "mkdir -p " + folder;
      const int ret = std::system(command.c_str());
      (void)ret;

      // print trajectory to file
      std::string path_x = "/tmp/plan/x.txt";
      std::ofstream file_x(path_x);
      for (int i = 0; i < N_STEP_; ++i) {
        file_x << x_ref.col(i).transpose() << std::endl;
      }
      file_x.close();
      std::string path_u = "/tmp/plan/u.txt";
      std::ofstream file_u(path_u);
      for (int i = 0; i < N_STEP_ - 1; ++i) {
        file_u << u_ref.col(i).transpose() << std::endl;
      }
      file_u.close();
    }
  }

  void jumpRoutine(const double& t_msec){

    double t0 = t_msec / 1000;
    int current_time_step = get_time_step_idx(t_msec);
    double com_z_cur = x_ref.col(current_time_step)(2);

    double T_down = 0.75;
    double T_pre = 1;
    double T_up = 0.75;

    double t_in = t0 + T_down + T_pre;
    double h_jump = 0.10;
    double g = 9.81;
    double v0_jump = std::sqrt(2 * g * h_jump);

    double T_jump = 2 * v0_jump / g;
    
    double T_total = T_jump + T_down + 2 * T_pre + T_up;
    if(t0 + T_total > T){
      stop_trajectory(t_msec);
      return;
    }
    int N_STEP_JUMP = static_cast<int>(T_total / dt_);



    double vz          = 0.0;
    double v_contact_z = 0.0;
  
    double z0_contact  = 0.0;

    double z_min       = 0.3;

    double z = com_z_cur;
    double z_contact = z0_contact;

    for (int t_step = 0; t_step < N_STEP_JUMP; ++t_step){
        double t = t0 + t_step * dt_;
         
        if (t < t0 + T_down){
          vz = -0.2;
          v_contact_z = 0.0;
        } else if (t >= t0 + T_down  && t < t0 + + T_down + T_pre){
          vz = 0.0;
          v_contact_z = 0.0;
        }else if (t >= t0 + T_down + T_pre  && t < t0 + T_down + T_pre + T_jump + (dt_)){  // jump time   dt_ introduced because the integration is discrete and at time t it could be still in landing 

          vz = v0_jump - g * (t - t_in);
          v_contact_z = vz;
        } else if (t >= t0 + T_down + T_pre + T_jump + (dt_)  && t < t0 + T_down + T_pre + T_jump + T_pre){
          vz = 0.0;
          v_contact_z = 0.0;
        }else if (t >= t0 + T_down + T_pre + T_jump + T_pre){
          vz = 0.2;
          v_contact_z = 0.0;
        }



      
        z += vz * dt_;
        // clamp if exceeds limits
        if (z < z_min ){
          vz = 0.0;
          z = z_min;
        } else if (z + vz * dt_ > com_z_cur && t >= t0 + T_down + T_pre + T_jump){
          vz = 0.0;
          z = com_z_cur;
        }


        // double z_contact = z0_contact + v_contact_z * t;
        z_contact += v_contact_z * dt_;
        z_contact = std::max(z_contact, 0.0);

        x_ref.col(current_time_step + t_step)(2)  = z;
        x_ref.col(current_time_step + t_step)(5)  = vz;

        x_ref.col(current_time_step + t_step)(8)  = z_contact;

        x_ref.col(current_time_step + t_step)(9)  = v_contact_z;    
    }


    if (log_plan_){
      // print trajectory to file
      std::string path_jump = "/tmp/plan/jump_traj.txt";
      std::ofstream file_jump(path_jump);
      file_jump << t_msec << " index " << current_time_step << std::endl;
      for (int i = 0; i < N_STEP_JUMP; ++i) {
        file_jump << x_ref.col(current_time_step + i).transpose() << std::endl;
      }
      file_jump.close();
    }

  }



  void stop_trajectory(const double& t_msec){

    double t0 = t_msec / 1000;
    int current_time_step = get_time_step_idx(t_msec);

    int N_CURR = static_cast<int>(t0 / dt_);

    double com_x_cur = x_ref.col(current_time_step)(0);
    double com_y_cur = x_ref.col(current_time_step)(1);
    double com_z_cur = x_ref.col(current_time_step)(2);
    double theta_cur = x_ref.col(current_time_step)(10);
    for (int t_step = N_CURR; t_step < N_STEP_; ++t_step){

        x_ref.col(t_step)(0)  = com_x_cur;
        x_ref.col(t_step)(1)  = com_y_cur;
        x_ref.col(t_step)(2)  = com_z_cur;

        x_ref.col(t_step)(3)  = 0.0;
        x_ref.col(t_step)(4)  = 0.0;
        x_ref.col(t_step)(5)  = 0.0;

        x_ref.col(t_step)(6)  = com_x_cur;
        x_ref.col(t_step)(7)  = com_y_cur;
        x_ref.col(t_step)(8)  = 0.0;

        x_ref.col(t_step)(9)  = 0.0;
        x_ref.col(t_step)(10) = theta_cur;
        x_ref.col(t_step)(11) = 0.0;
        x_ref.col(t_step)(12) = 0.0;  
    }
  }


  int get_time_step_idx(const double& t_msec) const{
   return static_cast<int>(std::llround(t_msec / 1000 / dt_));         // check the rounding when control timestep is not fixed
  }

  Eigen::MatrixXd get_xref_at_time_ms(const double& t_msec) const {
    int time_step = get_time_step_idx(t_msec);
    time_step = std::clamp(time_step, 0, N_STEP_ - 1);
    return x_ref.col(time_step);
  }
  Eigen::MatrixXd get_uref_at_time_ms(const double& t_msec) const {
    int time_step = get_time_step_idx(t_msec);
    time_step = std::clamp(time_step, 0, N_STEP_ - 2);
    return u_ref.col(time_step);
  }

}; 

} // end namespace labrob
