#pragma once

#include <iostream>
#include <fstream>

#include <Eigen/Dense>



namespace labrob {

class walkingPlanner {
  private:
    static constexpr int NX = 14; 
    static constexpr int NU = 9; 
    static constexpr int T = 6;              // in sec
    
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


  void offline_plan(const double& dt, bool log_plan, const Eigen::Vector3d& pcom){

    dt_ = dt;
    N_STEP_ = static_cast<int>(T / dt_);     //n. of timesteps
    log_plan_ = log_plan;

    x_ref.setZero(NX, N_STEP_);
    u_ref.setZero(NU, N_STEP_-1);


    double T_const = 2 * T / 3;                // fast trajectory T = 11; T_const = 3 * T / 11;
    double T_acc   = (T - T_const) / 2;

    double v_peak     = 1.2;
    double omega_peak = 0.0;

    double a_max = v_peak / T_acc;
    double alpha_max = omega_peak / T_acc;


    double vz          = 0.0;
    double v_contact_z = 0.0;
    double v           = 0.0;
    double omega       = 0.0;
    double theta0      = 0.0;

    double a           = 0.0;
    double alpha       = 0.0;

    double x0          = pcom(0);
    double y0          = pcom(1);
    double z0          = pcom(2);
    double z0_contact  = 0.0;

    double z_min       = 0.25;
    double z_max       = 0.42;
 
    double x = x0;
    double y = y0;
    double z = z0;

    double theta = theta0;


    for (int t_step = 0; t_step < N_STEP_; ++t_step){
      double t = t_step * dt_;


      // Bang-Bang acceleration profile
      int phase = 3; // default: after end
      if (t < T_acc)                  phase = 0;
      else if (t < T_acc + T_const)   phase = 1;
      else if (t < T)                 phase = 2;

      switch (phase) {
        case 0: // accel
          a =  a_max;
          v =  a_max * t;

          alpha =  alpha_max;
          omega =  alpha_max * t;
          break;

        case 1: // cruise
          a = 0.0;
          v = v_peak;

          alpha = 0.0;
          omega = omega_peak;
          break;

        case 2: { // decel
          double td = t - (T_acc + T_const);
          a = -a_max;
          v = v_peak - a_max * td;

          alpha = -alpha_max;
          omega = omega_peak - alpha_max * td;
          break;
        }

        default: // after end
          a = 0.0;
          v = 0.0;

          alpha = 0.0;
          omega = 0.0;
          break;
      }



      // unicycle equations
      double vx = v * cos(theta);
      double vy = v * sin(theta);
      
      x  += vx * dt_;
      y  += vy * dt_;
      theta += omega * dt_;
      

      z = std::clamp(z + vz * dt_, z_min, z_max);
      double z_contact = z0_contact + v_contact_z * t;

      if (z <= z_min || z >= z_max){
        vz = 0.0;
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


      x_ref.col(t_step)(13) = 0;            // jump state
    }


    for (int t_step = 0; t_step < N_STEP_ - 1; ++t_step){
      u_ref.col(t_step)(0) = 0.0;      // a
      u_ref.col(t_step)(1) = 0.0;      // ac_z
      u_ref.col(t_step)(2) = 0.0;      // alpha
      
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

  void jumpRoutine(const double& t_msec, const double h_jump){

    double t0 = t_msec / 1000;
    int current_time_step = get_time_step_idx(t_msec);
    double com_z_cur = x_ref.col(current_time_step)(2);


    double g = 9.81;
    double v0_jump = std::sqrt(2 * g * h_jump);
    double T_jump = 2 * v0_jump / g;


    double vz          = 0.0;
    double v_contact_z = 0.0;
    double z0_contact  = 0.0;

    // initial conditions
    double z_start_jump = com_z_cur;
    double z = com_z_cur;
    double z_contact = z0_contact;


    // down cubic poly params
    double disc = (v0_jump + 2 * vz) * (v0_jump + 2 * vz) - 6 * g * (z_start_jump - z);
    double T_down = ((v0_jump + 2 * vz) + std::sqrt(disc)) / g;

    double d_down = z;
    double c_down = vz * T_down;
    double b_down = - 0.5 * g * T_down * T_down;
    double a_down = (T_down / 3.0) * (v0_jump - vz + g * T_down);


    // up cubic poly params
    double v_in_up = (v0_jump - g * T_jump);
    double asc = (v_in_up + 2 * vz) * (v_in_up + 2 * vz) - 6 * g * (z - z_start_jump);
    double T_up = (- (v_in_up + 2 * vz) + std::sqrt(asc)) / g;

    double d_up = z_start_jump;
    double c_up = v_in_up * T_up;
    double b_up = 0.5 * g * T_up * T_up + T_up * (vz - v_in_up);
    double a_up = - (T_up / 3.0) * (vz - v_in_up + g * T_up);



    double t_in = t0 + T_down;
    double T_total = T_jump + T_down + T_up;
    double T_detachment = 0.05;
    if(t0 + T_total > T){
      stop_trajectory(t_msec);
      return;
    }
    int N_STEP_JUMP = static_cast<int>(T_total / dt_) + 1;


    // leg trajectory
    double h_leg_max = h_jump;            // or 0.4 for h_jump = 3.1 (max-height jump)
    double e_contact = z0_contact;
    double d_contact = 0.0;
    double c_contact = 16 * (h_leg_max - z0_contact);
    double b_contact = -32 * (h_leg_max - z0_contact);
    double a_contact = 16 * (h_leg_max - z0_contact);


    for (int t_step = 0; t_step < N_STEP_JUMP; ++t_step){
        double t = t0 + t_step * dt_;

        int jump_state = 0;
         
        if (t < t0 + T_down){
          double tau = (t - t0) / T_down;
          tau = std::clamp(tau, 0.0, 1.0);
          z = a_down * tau * tau * tau + b_down * tau * tau + c_down * tau + d_down;
          vz = (3 * a_down * tau * tau + 2 * b_down * tau + c_down) * 1 / T_down;
          z_contact = 0.0;
          v_contact_z = 0.0;

          jump_state = (t < t0 + T_down - T_detachment) ? 1: 2;      // pre-jump phase / ground-detachmant phase

        } else if (t >= t0 + T_down  && t < t0 + T_down + T_jump + (dt_)){  // jump time   dt_ introduced because the integration is discrete and at time t it could be still in landing 
          double tj = t - t_in;
          z  = z_start_jump + v0_jump * tj - 0.5 * g * tj * tj;
          vz = v0_jump - g * tj;

          double tau = (t - (t0 + T_down)) / T_jump;
          tau = std::clamp(tau, 0.0, 1.0);

          z_contact = a_contact * tau * tau * tau * tau + b_contact * tau * tau * tau + c_contact * tau * tau + d_contact * tau + e_contact;
          z_contact = std::max(z_contact, 0.0);
          v_contact_z = (4 * a_contact * tau * tau * tau + 3 * b_contact * tau * tau + 2 * c_contact * tau + d_contact) * 1 / T_jump;

          jump_state = 3;      // flight phase

        } else if (t >= t0 + T_down + T_jump + (dt_)){
          double tau = (t - (t0 + T_down + T_jump)) / T_up;
          tau = std::clamp(tau, 0.0, 1.0);
          z = a_up * tau * tau * tau + b_up * tau * tau + c_up * tau + d_up;
          vz = (3 * a_up * tau * tau + 2 * b_up * tau + c_up) * 1 / T_up;
          z_contact = 0.0;
          v_contact_z = 0.0;

          jump_state = 4;      // post-jump phase

        }

        x_ref.col(current_time_step + t_step)(2)  = z;
        x_ref.col(current_time_step + t_step)(5)  = vz;

        x_ref.col(current_time_step + t_step)(8)  = z_contact;

        x_ref.col(current_time_step + t_step)(9)  = v_contact_z;    

        x_ref.col(current_time_step + t_step)(13)  = jump_state;       // jump state
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
        double t = t0 + t_step * dt_;

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

        x_ref.col(t_step)(13) = 0;         // jump state
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
  int get_jump_phase_at_time_ms(const double& t_msec) const {
    int time_step = get_time_step_idx(t_msec);
    time_step = std::clamp(time_step, 0, N_STEP_ - 1);
    return x_ref.col(time_step)(13);
  }

}; 

} // end namespace labrob
