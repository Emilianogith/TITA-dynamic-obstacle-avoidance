#include <MPC.hpp>


void labrob::MPC::solve(Eigen::Vector<double, NX> x0, Eigen::Vector3d curr_zmp_vel){

  // ------------------------------------------------------------- //
  // dynamics ---------------------------------------------------- //

  auto x_dot = [this](const VectorX &x, const VectorU &u) -> VectorX
  {
      return A * x + B * u;
  };

  auto f = [this](const VectorX &x, const VectorU &u) -> VectorX
  {
      return x + Δ * (A * x + B * u);
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
      Eigen::Matrix<double, NY, 1> vcom = Cv_com * x;
      Eigen::Matrix<double, NY, 1> vcom_error = vcom;

      Eigen::Matrix<double, NY, 1> zmp = C_zmp * x;
      Eigen::Matrix<double, NY, 1> zmp_error = zmp - zmp_ref.col(i);
      Eigen::Matrix<double, 1, 1> ret;
      ret(0,0) = w_z * zmp_error.squaredNorm() + w_zd * u.squaredNorm() + w_cd * vcom_error.squaredNorm();
      return ret;
  };

  auto Lx = [this](const VectorX &x, const VectorU &u, const int &i) -> VectorX
  {
      Eigen::Matrix<double, NY, 1> vcom = Cv_com * x;
      Eigen::Matrix<double, NY, 1> vcom_error = vcom;

      Eigen::Matrix<double, NY, 1> zmp = C_zmp * x;
      Eigen::Matrix<double, NY, 1> zmp_error = zmp - zmp_ref.col(i);
      return (2 * w_z * C_zmp.transpose() * zmp_error + 2 * w_cd * Cv_com.transpose() * vcom_error).transpose();
  };

  auto Lu = [this](const VectorX &x, const VectorU &u, const int &i) -> VectorU
  {
      return 2 * w_zd * u;
  };

  auto Lxx = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NX, NX>
  {
      Eigen::Matrix<double, NY, 1> vcom = Cv_com * x;
      Eigen::Matrix<double, NY, 1> vcom_error = vcom;

      Eigen::Matrix<double, NY, 1> zmp = C_zmp * x;
      Eigen::Matrix<double, NY, 1> zmp_error = zmp - zmp_ref.col(i);
      return 2 * w_z * C_zmp.transpose() * C_zmp + 2 * w_cd * Cv_com.transpose() * Cv_com;
  };

  auto Luu = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NU, NU>
  {
      return 2 * w_zd * Eigen::Matrix<double, NU, NU>::Identity();
  };

  auto Lux = [this](const VectorX &x, const VectorU &u, const int &i) -> Eigen::Matrix<double, NU, NX>
  {
      return Eigen::Matrix<double, NU, NX>::Zero();
  };

  // ------------------------------------------------------------- //
  // terminal cost ----------------------------------------------- //

  auto L_ter = [this](const VectorX &x) -> Eigen::Matrix<double, 1, 1>
  {
      return Eigen::Matrix<double, 1, 1>::Zero();

      // Eigen::Matrix<double, NY, NX> Cv_com;
      // Cv_com << 0,1,0;
      // Eigen::Matrix<double, NY, 1> v_com = Cv_com * x;
      // Eigen::Matrix<double, NY, 1> v_com_error = v_com;
      // Eigen::Matrix<double, 1, 1> ret;
      // ret(0,0) = w_zd * v_com_error.squaredNorm();
      // return ret;
  };

  auto L_terx = [this](const VectorX &x) -> VectorX
  {
      return VectorX::Zero();
      // Eigen::Matrix<double, NY, NX> Cv_com;
      // Cv_com << 0,1,0;
      // Eigen::Matrix<double, NY, 1> v_com = Cv_com * x;
      // Eigen::Matrix<double, NY, 1> v_com_error = v_com;
      // return (2 * w_zd * Cv_com.transpose() * v_com_error).transpose();;
  };

  auto L_terxx = [this](const VectorX &x) -> Eigen::Matrix<double, NX, NX>
  {
      return Eigen::Matrix<double, NX, NX>::Zero();
      // Eigen::Matrix<double, NY, NX> Cv_com;
      // Cv_com << 0,1,0;
      // Eigen::Matrix<double, NY, 1> v_com = Cv_com * x;
      // Eigen::Matrix<double, NY, 1> v_com_error = v_com;
      // return 2 * w_zd * Cv_com.transpose() * Cv_com;;
  };

  // ------------------------------------------------------------- //
  // inequality constraints -------------------------------------- //

  auto g = [this](const VectorX &x, const int &i) -> Eigen::Matrix<double, NC, 1>
  {
      Eigen::Matrix<double, NY, 1> zmp = C_zmp * x;
      Eigen::Matrix<double, NY, 1> zmp_error = zmp - zmp_ref.col(i);

      Eigen::Matrix<double, NC, 1> g;
      double constraint_half_width = 0.05;
      
      // // upper boundary
      // g(0,0) = - (constraint_half_width - zmp_error(0,0));
      // g(1,0) = - (constraint_half_width - zmp_error(1,0));
      // g(2,0) = - (constraint_half_width - zmp_error(2,0));

      // // lower boundary
      // g(3,0) = - (zmp_error(0,0) + constraint_half_width);
      // g(4,0) = - (zmp_error(1,0) + constraint_half_width);
      // g(5,0) = - (zmp_error(2,0) + constraint_half_width);



      g(0,0) = - (constraint_half_width - zmp_error(0,0));
      g(1,0) = - (zmp_error(0,0) + constraint_half_width);
      

      return g;
  };

  auto gx = [this](const VectorX &x, const int &i) -> Eigen::Matrix<double, NC, NX>
  {
      Eigen::Matrix<double, NC, NX> ret;
      ret << C_zmp, -C_zmp;
      return ret;
  };

  auto gu = [this](const VectorX &x, const int &i) -> Eigen::Matrix<double, NC, NU>
  {
      return Eigen::Matrix<double, NC, NU>::Zero();
  };

  // ------------------------------------------------------------- //
  // terminal constraint ----------------------------------------- //

  auto h_ter = [this](const VectorX &x) -> Eigen::Matrix<double, NY, 1>
  {
      //  Eigen::Matrix<double, NY, 1> H = (C_com - C_zmp) * x;
      //  H.row(NY - 1).setZero();
      //  return H;
      return (C_com - C_zmp) * x;
  };

  auto h_terx = [this](const VectorX &x) -> Eigen::Matrix<double, NY, NX>
  {
      // Eigen::Matrix<double, NY, NX> H = C_com - C_zmp;
      // H.row(NY - 1).setZero();
      // return H;
      return C_com - C_zmp;
  };


  // initialize problem
  auto solver = DdpSolver<NX, NU, NY, NC, NH>(
    f, fx, fu,
    L, Lx, Lu, Lxx, Luu, Lux,
    L_ter, L_terx, L_terxx,
    h_ter, h_terx,
    g, gx, gu,
    SOLVER_MAX_ITER); 



  std::array<Eigen::Vector<double, NX>, NH+1> x_traj;
  x_traj[0] = x0;
  std::array<Eigen::Vector<double, NU>, NH  > u_traj;
  std::array<Eigen::Vector<double, NX>, NH  > xdot_traj;

  // set warm-start trajectories
  std::array<Eigen::Vector<double, NX>, NH+1> x_guess;
  for (int i = 0; i < NH+1; ++i)
    x_guess[i] = x0;
  std::array<Eigen::Vector<double, NU>, NH> u_guess;
  for (int i = 0; i < NH; ++i)
    u_guess[i].setZero();
  solver.set_initial_state(x0);
  solver.set_x_warmstart(x_guess);
  solver.set_u_warmstart(u_guess);

  // solve DDP
  solver.solve();

  // integrate dynamics
  for (int i = 0; i < NH; ++i) {
    VectorX x_i = x_traj[i];
    VectorU u_i = solver.u[i];
    x_traj[i+1] = f(x_i, u_i);
    xdot_traj[i] = x_dot(x_i, u_i);
    u_traj[i] = u_i;
  }

  const auto& x_prediction = x_traj[1];
  const auto& xdot_prediction = xdot_traj[0];
  const auto& u_prediction = u_traj[0];


  // pos_com_ = Eigen::Vector3d(x_prediction(0), x_prediction(3), x_prediction(6));
  // vel_com_ = Eigen::Vector3d(x_prediction(1), x_prediction(4), x_prediction(7));
  // acc_com_ = Eigen::Vector3d(xdot_prediction(1), xdot_prediction(4), xdot_prediction(7));
  // pos_zmp_ = Eigen::Vector3d(x_prediction(2), x_prediction(5), x_prediction(8));
  // vel_zmp_ = Eigen::Vector3d(u_prediction(0), u_prediction(1), u_prediction(2));
  // acc_zmp_ = (vel_zmp_ - curr_zmp_vel) / Δ;


  pos_com_ = Eigen::Vector3d(x_prediction(0), 0.0, 0.0);
  vel_com_ = Eigen::Vector3d(x_prediction(1), 0.0, 0.0);
  acc_com_ = Eigen::Vector3d(xdot_prediction(1), 0.0, 0.0);
  pos_zmp_ = Eigen::Vector3d(x_prediction(2), 0.0, 0.0);
  vel_zmp_ = Eigen::Vector3d(u_prediction(0), 0.0, 0.0);
  acc_zmp_ = (vel_zmp_ - curr_zmp_vel) / Δ;


  // logs
  if (record_logs){
    // create folder if it does not exist
    std::string folder = "/tmp/mpc_data";
    std::string command = "mkdir -p " + folder;
    system(command.c_str());

    // print trajectory to file
    std::ofstream file_x("/tmp/mpc_data/x.txt");
    for (int i = 0; i < NH+1; ++i) {
      file_x << x_traj[i].transpose() << std::endl;
    }
    file_x.close();
    std::ofstream file_u("/tmp/mpc_data/u.txt");
    for (int i = 0; i < NH; ++i) {
      file_u << u_traj[i].transpose() << std::endl;
    }
    file_u.close();

    record_logs = false;
  }
}












