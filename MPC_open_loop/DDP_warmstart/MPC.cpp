#include <MPC.hpp>

void labrob::MPC::initSolver(Eigen::Vector<double, NX> x0) {
    // ------------------------------------------------------------- //
    // dynamics ---------------------------------------------------- //

    auto x_dot = [this](const VectorX &x, const VectorU &u) -> VectorX
    {
        return A * x + B * u + c;
    };

    auto f = [this](const VectorX &x, const VectorU &u) -> VectorX
    {
        return x + Δ * (A * x + B * u + c);
    };

    auto fx = [this](const VectorX &x, const VectorU &u) -> Eigen::Matrix<double, NX, NX>
    {
        return Eigen::Matrix<double, NX, NX>::Identity() + Δ * A;
    };

    auto fu = [this](const VectorX &x, const VectorU &u) -> Eigen::Matrix<double, NX, NU>
    {
        return Δ * B;
    };

    // ------------------------------------------------------------- //
    // running cost ------------------------------------------------ //
    
    auto L = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, 1, 1>
    { 
        double hcom = (C_com * x)(2);
        double hcom_error = hcom - pcom_ref(2,i);
        double hvcom = (Cv_com * x)(2);
        double hvcom_error = hvcom - 0.0;

        Eigen::Vector2d f_c_xy = u.segment<2>(2);
        Eigen::Vector2d f_c_xy_error = f_c_xy;

        double f_c_z_error = u(4) - m*grav;
        
        Eigen::Matrix<double, NY-1, 1> vcom_2D = Cv_com.topRows(2) * x;
        Eigen::Matrix<double, NY-1, 1> vcom_error_2D = vcom_2D;

        Eigen::Matrix<double, NY_zmp, 1> vzmp_2D = Cv_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> vzmp_error_2D = vzmp_2D;

        Eigen::Matrix<double, NY_zmp, 1> zmp_2D = C_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> zmp_error_2D = zmp_2D - zmp_ref.col(i);
        Eigen::Matrix<double, 1, 1> ret;
        ret(0,0) = w_z * zmp_error_2D.squaredNorm() 
                  + w_zd * vzmp_error_2D.squaredNorm() 
                  + w_cd * vcom_error_2D.squaredNorm()
                  + w_acc * u.segment<2>(0).squaredNorm()
                  + w_fxy * f_c_xy_error.squaredNorm()
                  + w_fz * f_c_z_error * f_c_z_error
                  + w_h * hcom_error * hcom_error
                  + w_vh * hvcom_error * hvcom_error;
        return ret;
    };

    auto Lx = [this](const VectorX &x, const VectorU &u, const int &i) -> VectorX
    {
        double hcom = (C_com * x)(2);
        double hcom_error = hcom - pcom_ref(2,i);
        double hvcom = (Cv_com * x)(2);
        double hvcom_error = hvcom - 0.0;
        
        Eigen::Matrix<double, NY-1, 1> vcom_2D = Cv_com.topRows(2) * x;
        Eigen::Matrix<double, NY-1, 1> vcom_error_2D = vcom_2D;

        Eigen::Matrix<double, NY_zmp, 1> vzmp_2D = Cv_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> vzmp_error_2D = vzmp_2D;

        Eigen::Matrix<double, NY_zmp, 1> zmp_2D = C_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> zmp_error_2D = zmp_2D - zmp_ref.col(i);
        return 2 * w_z * C_zmp.transpose() * zmp_error_2D 
              + 2 * w_zd * Cv_zmp.transpose() * vzmp_error_2D 
              + 2 * w_cd * Cv_com.topRows(2).transpose() * vcom_error_2D
              + 2 * w_h * C_com.row(2).transpose() * hcom_error
              + 2 * w_vh * Cv_com.row(2).transpose() * hvcom_error;
    };

    auto Lu = [this](const VectorX &x, const VectorU &u, const int &i) -> VectorU
    {
        Eigen::Vector2d f_c_xy = u.segment<2>(2);
        Eigen::Vector2d f_c_xy_error = f_c_xy;

        double hcom = (C_com * x)(2);
        double hcom_error = hcom - pcom_ref(2,i);
        double hvcom = (Cv_com * x)(2);
        double hvcom_error = hvcom - 0.0;

        double f_c_z_error = u(4) - m*grav;
        
        VectorU ret = VectorU::Zero();
        ret.segment<2>(0) = 2 * w_acc * u.segment<2>(0);
        ret.segment<2>(2) = 2 * w_fxy * f_c_xy_error;
        ret(4) = 2 * w_fz * f_c_z_error;
        return ret;
    };

    auto Lxx = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NX, NX>
    {
        return 2 * w_z * C_zmp.transpose() * C_zmp 
        + 2 * w_zd * Cv_zmp.transpose() * Cv_zmp
        + 2 * w_cd * Cv_com.topRows(2).transpose() * Cv_com.topRows(2)
        + 2 * w_h * C_com.row(2).transpose() * C_com.row(2)
        + 2 * w_vh * Cv_com.row(2).transpose() * Cv_com.row(2);
    };


    auto Luu = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NU, NU>
    {
        Eigen::Matrix<double, NU, NU> ret = Eigen::Matrix<double, NU, NU>::Zero();
        ret.block<2,2>(0,0)   = 2 * w_acc * Eigen::Matrix2d::Identity(); // acc
        ret.block<2,2>(2,2)   = 2 * w_fxy * Eigen::Matrix2d::Identity(); // f_xy
        ret(4,4)   = 2 * w_fz; // f_z
        return ret;
    };

    auto Lux = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NU, NX>
    {
        return Eigen::Matrix<double, NU, NX>::Zero();
    };


    // ------------------------------------------------------------- //
    // terminal cost ----------------------------------------------- //

    auto L_ter = [this](const VectorX &x) -> Eigen::Matrix<double, 1, 1>
    {
        double hcom = (C_com * x)(2);
        double hcom_error = hcom - pcom_ref(2,NH);
        double hvcom = (Cv_com * x)(2);
        double hvcom_error = hvcom - 0.0;

        Eigen::Matrix<double, NY-1, 1> vcom_2D = Cv_com.topRows(2) * x;
        Eigen::Matrix<double, NY-1, 1> vcom_error_2D = vcom_2D;

        Eigen::Matrix<double, NY_zmp, 1> vzmp_2D = Cv_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> vzmp_error_2D = vzmp_2D;

        Eigen::Matrix<double, NY_zmp, 1> zmp_2D = C_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> zmp_error_2D = zmp_2D - zmp_ref.col(NH);
        Eigen::Matrix<double, 1, 1> ret;
        ret(0,0) = w_ter * (w_z * zmp_error_2D.squaredNorm() + w_zd * vzmp_error_2D.squaredNorm() 
        + w_cd * vcom_error_2D.squaredNorm()
        + w_h * hcom_error * hcom_error
        + w_vh * hvcom_error * hvcom_error);
        return ret;
    };

    auto L_terx = [this](const VectorX &x) -> VectorX
    {
        double hcom = (C_com * x)(2);
        double hcom_error = hcom - pcom_ref(2,NH);
        double hvcom = (Cv_com * x)(2);
        double hvcom_error = hvcom - 0.0;

        Eigen::Matrix<double, NY-1, 1> vcom_2D = Cv_com.topRows(2) * x;
        Eigen::Matrix<double, NY-1, 1> vcom_error_2D = vcom_2D;

        Eigen::Matrix<double, NY_zmp, 1> vzmp_2D = Cv_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> vzmp_error_2D = vzmp_2D;

        Eigen::Matrix<double, NY_zmp, 1> zmp_2D = C_zmp * x;
        Eigen::Matrix<double, NY_zmp, 1> zmp_error_2D = zmp_2D - zmp_ref.col(NH);
        return w_ter * (2 * w_z * C_zmp.transpose() * zmp_error_2D 
              + 2 * w_zd * Cv_zmp.transpose() * vzmp_error_2D 
              + 2 * w_cd * Cv_com.topRows(2).transpose() * vcom_error_2D
              + 2 * w_h * C_com.row(2).transpose() * hcom_error
              + 2 * w_vh * Cv_com.row(2).transpose() * hvcom_error);
    };

    auto L_terxx = [this](const VectorX &x) -> Eigen::Matrix<double, NX, NX>
    {
        return w_ter * (2 * w_z * C_zmp.transpose() * C_zmp 
        + 2 * w_zd * Cv_zmp.transpose() * Cv_zmp
        + 2 * w_cd * Cv_com.topRows(2).transpose() * Cv_com.topRows(2)
        + 2 * w_h * C_com.row(2).transpose() * C_com.row(2)
        + 2 * w_vh * Cv_com.row(2).transpose() * Cv_com.row(2));
    };


    // ------------------------------------------------------------- //
    // inequality constraints -------------------------------------- //

    auto g = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NC, 1>
    {
        // Eigen::Matrix<double, NY-1, 1> zmp = C_zmp * x;
        // Eigen::Matrix<double, NY-1, 1> zmp_error = zmp - zmp_ref.col(i).topRows(2);

        // Eigen::Matrix<double, NC, 1> g;
        // double constraint_half_width = 0.05;
        
        // // // upper boundary
        // // g(0,0) = - (constraint_half_width - zmp_error(0,0));
        // // g(1,0) = - (constraint_half_width - zmp_error(1,0));
        // // g(2,0) = - (constraint_half_width - zmp_error(2,0));

        // // // lower boundary
        // // g(3,0) = - (zmp_error(0,0) + constraint_half_width);
        // // g(4,0) = - (zmp_error(1,0) + constraint_half_width);
        // // g(5,0) = - (zmp_error(2,0) + constraint_half_width);



        // g(0,0) = - (constraint_half_width - zmp_error(0,0));
        // g(1,0) = - (constraint_half_width - zmp_error(1,0));
        // g(2,0) = - (zmp_error(0,0) + constraint_half_width);
        // g(3,0) = - (zmp_error(1,0) + constraint_half_width);
        

        // return g;
        return Eigen::Matrix<double, NC, 1>::Zero();
    };

    auto gx = [this](const VectorX &x, const int &i) -> Eigen::Matrix<double, NC, NX>
    {
        // Eigen::Matrix<double, NC, NX> ret;
        // ret << C_zmp, -C_zmp;
        // return ret;
        return Eigen::Matrix<double, NC, NX>::Zero();
    };

    auto gu = [this](const VectorX &x, const int &i) -> Eigen::Matrix<double, NC, NU>
    {
        return Eigen::Matrix<double, NC, NU>::Zero();
    };

    // ------------------------------------------------------------- //
    // terminal constraint ----------------------------------------- //

    auto h_ter = [this](const VectorX &x) -> Eigen::Matrix<double, NY_zmp, 1>
    {
        Eigen::Matrix<double, NY_zmp, NX> H = Eigen::Matrix<double, NY_zmp, NX>::Zero();
        H = C_com.topRows<2>() - C_zmp;
        return H * x;
    };

    auto h_terx = [this](const VectorX &x) -> Eigen::Matrix<double, NY_zmp, NX>
    {
        Eigen::Matrix<double, NY_zmp, NX> H = Eigen::Matrix<double, NY_zmp, NX>::Zero();
        H = C_com.topRows<2>() - C_zmp;
        return H;
    };


    auto h = [this](const VectorX &x, const VectorU &u) -> Eigen::Matrix<double, NE, 1>
    {
      Eigen::Vector3d f_c = u.segment<3>(2);
      Eigen::Vector3d pcom = C_com * x;
      Eigen::Vector3d pc; 
      pc.segment<2>(0) = C_zmp * x; 
      pc(2) = z_c;

      Eigen::Matrix<double, NE, 1> ret = Eigen::Matrix<double, NE, 1>::Zero();
      // ret.block(0,0,3,1) = (pc - pcom).cross(f_c);  // -[f_c]_x (C_pc - C_com) x =   [pc - pcom]_x (f_c)
      ret(3,0) = f_c(2) - m * grav; // f_z = m*g

      return ret;
    };

    auto hx = [this](const VectorX &x, const VectorU &u) -> Eigen::Matrix<double, NE, NX>
    {
      Eigen::Vector3d f_c = u.segment<3>(2);
      const Eigen::Matrix3d Sfc = pinocchio::skew(f_c);

      // Build C_pc: Jacobian of pc wrt x (z is constant → third row = 0)
      Eigen::Matrix<double,3,NX> C_pc = Eigen::Matrix<double,3,NX>::Zero();
      C_pc.topRows<2>() = C_zmp;


      Eigen::Matrix<double, NE, NX> H = Eigen::Matrix<double, NE, NX>::Zero();
      // H.topRows(3) = -Sfc * (C_pc - C_com);

      return H;
    };

    auto hu = [this](const VectorX &x, const VectorU &u) -> Eigen::Matrix<double, NE, NU>
    {
      Eigen::Vector3d pcom = C_com * x;
      Eigen::Vector3d pc; 
      pc.segment<2>(0) = C_zmp * x; 
      pc(2) = z_c;

      const Eigen::Vector3d r = pc - pcom;
      const Eigen::Matrix3d S_r = pinocchio::skew(r);

      Eigen::Matrix<double, NE, NU> Hu = Eigen::Matrix<double, NE, NU>::Zero();
      // Hu.block<3,3>(0,2) = S_r;
      Hu(3,4) = 1.0;

      return Hu;
    };

    solver_ptr_ = std::make_unique<DdpSolver<NX, NU, NE, NY_zmp, NC, NH>>(
        f, fx, fu,
        L, Lx, Lu, Lxx, Luu, Lux,
        L_ter, L_terx, L_terxx,
        h_ter, h_terx,
        h, hx, hu,      // h
        g, gx, gu,      // g
        true,
        SOLVER_MAX_ITER,
        1.0,
        1e-1,
        0.5);


    // set warm-start trajectories
    std::array<Eigen::Vector<double, NX>, NH+1> x_guess;
    for (int i = 0; i < NH+1; ++i)
      x_guess[i] = x0;
    std::array<Eigen::Vector<double, NU>, NH> u_guess;
    for (int i = 0; i < NH; ++i)
      // u_guess[i].setZero();
      u_guess[i] = u0;

    solver_ptr_->set_x_warmstart(x_guess);
    solver_ptr_->set_u_warmstart(u_guess);

    std::cout << "solver initialized " << std::endl;

}

void labrob::MPC::solve(Eigen::Vector<double, NX> x0){


  auto x_dot = [this](const VectorX &x, const VectorU &u) -> VectorX
  {
      return A * x + B * u + c;
  };

  auto f = [this](const VectorX &x, const VectorU &u) -> VectorX
  {
      return x + Δ * (A * x + B * u + c);
  };




  std::array<Eigen::Vector<double, NX>, NH+1> x_traj;
  x_traj[0] = x0;
  std::array<Eigen::Vector<double, NU>, NH  > u_traj;
  std::array<Eigen::Vector<double, NX>, NH  > xdot_traj;


  solver_ptr_->set_initial_state(x0);


  // solve DDP
  solver_ptr_->solve();

  // integrate dynamics
  for (int i = 0; i < NH; ++i) {
    VectorX x_i = x_traj[i];
    VectorU u_i = solver_ptr_->u[i];
    x_traj[i+1] = f(x_i, u_i);
    xdot_traj[i] = x_dot(x_i, u_i);
    u_traj[i] = u_i;
  }

  std::cout << "u_traj " << u_traj[0] << std::endl;

  const auto& x_prediction = x_traj[1];
  const auto& xdot_prediction = xdot_traj[1];
  const auto& u_prediction = u_traj[0];


  pos_com_ = Eigen::Vector3d(x_prediction(0), x_prediction(1), x_prediction(2));
  vel_com_ = Eigen::Vector3d(x_prediction(3), x_prediction(4), x_prediction(5));
  acc_com_ = Eigen::Vector3d(xdot_prediction(3), xdot_prediction(4), xdot_prediction(5));
  pos_zmp_ = Eigen::Vector3d(x_prediction(6), x_prediction(7), 0.0);
  vel_zmp_ = Eigen::Vector3d(x_prediction(8), x_prediction(9), 0.0);
  acc_zmp_ = Eigen::Vector3d(u_prediction(0), u_prediction(1), 0.0);






  // logs
  if (record_logs){
    // create folder if it does not exist
    std::string folder = "/tmp/mpc_data/" + std::to_string(t_msec);
    std::string command = "mkdir -p " + folder;
    system(command.c_str());

    // print trajectory to file
    std::string path_x = "/tmp/mpc_data/" + std::to_string(t_msec) + "/x.txt";
    std::ofstream file_x(path_x);
    for (int i = 0; i < NH+1; ++i) {
      file_x << x_traj[i].transpose() << std::endl;
    }
    file_x.close();
    std::string path_u = "/tmp/mpc_data/" + std::to_string(t_msec) + "/u.txt";
    std::ofstream file_u(path_u);
    for (int i = 0; i < NH; ++i) {
      file_u << u_traj[i].transpose() << std::endl;
    }
    file_u.close();

    record_logs = false;
  }
}












