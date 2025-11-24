#include <MPC.hpp>

void labrob::MPC::solve(Eigen::Vector<double, N_STATE> x0){

  //QP-z
  // ------------------------------------------------------------- //
  // dynamics ---------------------------------------------------- //

  auto x_dot_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> QP_Z::VectorX
  { return Qp_z.A * x + Qp_z.B * u + Qp_z.c;};

  auto f_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> QP_Z::VectorX
  { return x + Δ * (Qp_z.A * x + Qp_z.B * u + Qp_z.c);};

  auto fx_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_Z::NX, QP_Z::NX>
  { return Eigen::Matrix<double, QP_Z::NX, QP_Z::NX>::Identity() + Δ * Qp_z.A;};

  auto fu_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_Z::NX, QP_Z::NU>
  { return Δ * Qp_z.B;};

  // ------------------------------------------------------------- //
  // running cost ------------------------------------------------ //
  
  auto L_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> Eigen::Matrix<double, 1, 1>
  { 
      double hcom = Qp_z.C_com * x;
      double hcom_error = hcom - pcom_ref(2,i);
      double hvcom = Qp_z.Cv_com * x;
      double hvcom_error = hvcom - 0.0;

      double fz_error = u(0) - m* grav;
      
      Eigen::Matrix<double, 1, 1> ret;
      ret(0,0) = + w_h * hcom_error * hcom_error
                 + w_vh * hvcom_error * hvcom_error
                 + w_fc * fz_error * fz_error;
      return ret;
  };

  auto Lx_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> QP_Z::VectorX
  {
      double hcom = Qp_z.C_com * x;
      double hcom_error = hcom - pcom_ref(2,i);
      double hvcom = Qp_z.Cv_com * x;
      double hvcom_error = hvcom - 0.0;
      
      return 2 * w_h * Qp_z.C_com.transpose() * hcom_error
             + 2 * w_vh * Qp_z.Cv_com.transpose() * hvcom_error;
  };

  auto Lu_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> QP_Z::VectorU
  {
      double fz_error = u(0) - m* grav;
      QP_Z::VectorU ret = QP_Z::VectorU::Zero();
      ret(0) = 2 * w_fc * fz_error;
      return ret;
  };

  auto Lxx_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_Z::NX, QP_Z::NX>
  {
      return 2 * w_h * Qp_z.C_com.transpose() * Qp_z.C_com
            + 2 * w_vh * Qp_z.Cv_com.transpose() * Qp_z.Cv_com;
  };

  auto Luu_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_Z::NU, QP_Z::NU>
  {
      Eigen::Matrix<double, QP_Z::NU, QP_Z::NU> ret = Eigen::Matrix<double, QP_Z::NU, QP_Z::NU>::Zero();
      ret(0,0)   = 2 * w_fc ; 
      return ret;
  };

  auto Lux_z = [this](const QP_Z::VectorX &x, const QP_Z::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_Z::NU, QP_Z::NX>
  {
      return Eigen::Matrix<double, QP_Z::NU, QP_Z::NX>::Zero();
  };

  // ------------------------------------------------------------- //
  // terminal cost ----------------------------------------------- //

  auto L_ter_z = [this](const QP_Z::VectorX &x) -> Eigen::Matrix<double, 1, 1>
  {
      double hcom = Qp_z.C_com * x;
      double hcom_error = hcom - pcom_ref(2,NH);
      double hvcom = Qp_z.Cv_com * x;
      double hvcom_error = hvcom - 0.0;

      Eigen::Matrix<double, 1, 1> ret;
      ret(0,0) = w_h * hcom_error * hcom_error
                + w_vh * hvcom_error * hvcom_error;
      return ret;
  };

  auto L_terx_z = [this](const QP_Z::VectorX &x) -> QP_Z::VectorX
  {
      double hcom = Qp_z.C_com * x;
      double hcom_error = hcom - pcom_ref(2,NH);
      double hvcom = Qp_z.Cv_com * x;
      double hvcom_error = hvcom - 0.0;

      return 2 * w_h * Qp_z.C_com.transpose() * hcom_error
             + 2 * w_vh * Qp_z.Cv_com.transpose() * hvcom_error;
  };

  auto L_terxx_z = [this](const QP_Z::VectorX &x) -> Eigen::Matrix<double, QP_Z::NX, QP_Z::NX>
  {
      return 2 * w_h * Qp_z.C_com.transpose() * Qp_z.C_com
            + 2 * w_vh * Qp_z.Cv_com.transpose() * Qp_z.Cv_com;
  };

  // // terminal constraint ----------------------------------------- //
  // auto h_ter_z = [this](const QP_Z::VectorX &x) -> Eigen::Matrix<double, QP_Z::NY, 1>
  // {
  //     Eigen::Matrix<double, QP_Z::NY, QP_Z::NX> H = Eigen::Matrix<double, QP_Z::NY, QP_Z::NX>::Zero();
  //     H = Qp_z.C_com;
  //     Eigen::Matrix<double, QP_Z::NY, 1> hz = Eigen::Matrix<double, QP_Z::NY, 1>::Zero();
  //     hz(0,0) = 0.4;
  //     return H * x - hz;
  // };

  // auto h_terx_z = [this](const QP_Z::VectorX &x) -> Eigen::Matrix<double, QP_Z::NY, QP_Z::NX>
  // {
  //     Eigen::Matrix<double, QP_Z::NY, QP_Z::NX> H = Eigen::Matrix<double, QP_Z::NY, QP_Z::NX>::Zero();
  //     H = Qp_z.C_com;
  //     return H;
  // };



    // initialize problem
    auto solver_z = DdpSolver<QP_Z::NX, QP_Z::NU, 0, QP_Z::NY, 0, NH>(
    f_z, fx_z, fu_z,
    L_z, Lx_z, Lu_z, Lxx_z, Luu_z, Lux_z,
    L_ter_z, L_terx_z, L_terxx_z,
    nullptr, nullptr,
    nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr,
    true,
    SOLVER_MAX_ITER,
    1.0,
    1e-1,
    0.5); 


  Eigen::Vector<double, QP_Z::NX> x0_z = Eigen::Vector<double, QP_Z::NX>::Zero();
  x0_z(0) = x0(2);
  x0_z(1) = x0(5);

  std::array<Eigen::Vector<double, QP_Z::NX>, NH+1> z_traj;
  z_traj[0] = x0_z;
  std::array<Eigen::Vector<double, QP_Z::NU>, NH  > fz_traj;
  std::array<Eigen::Vector<double, QP_Z::NX>, NH  > zdot_traj;

  // set warm-start trajectories
  std::array<Eigen::Vector<double, QP_Z::NX>, NH+1> z_guess;
  for (int i = 0; i < NH+1; ++i)
    z_guess[i] = x0_z;
  std::array<Eigen::Vector<double, QP_Z::NU>, NH> fz_guess;
  for (int i = 0; i < NH; ++i)
    fz_guess[i] = Qp_z.u_prev;

  solver_z.set_initial_state(x0_z);
  solver_z.set_x_warmstart(z_guess);
  solver_z.set_u_warmstart(fz_guess);

  // solve DDP
  solver_z.solve();

  // integrate dynamics
  for (int i = 0; i < NH; ++i) {
    QP_Z::VectorX x_i = z_traj[i];
    QP_Z::VectorU u_i = solver_z.u[i];
    z_traj[i+1] = f_z(x_i, u_i, i);
    zdot_traj[i] = x_dot_z(x_i, u_i, i);
    fz_traj[i] = u_i;
  }

  // std::cout << "fz_traj " << fz_traj[0] << std::endl;

  const auto& z_prediction = z_traj[1];
  const auto& zdot_prediction = zdot_traj[1];
  const auto& fz_prediction = fz_traj[0];

  Qp_z.u_prev = fz_prediction;





  //QP-xy
  // dynamics ---------------------------------------------------- //

  auto x_dot_xy = [this, &z_traj, &zdot_traj](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> QP_XY::VectorX
  {
      Eigen::Matrix<double, QP_XY::NX, QP_XY::NX> A = Eigen::Matrix<double, QP_XY::NX, QP_XY::NX>::Zero();
      Eigen::Matrix<double, QP_XY::NX, QP_XY::NU> B = Eigen::Matrix<double, QP_XY::NX, QP_XY::NU>::Zero();
      Eigen::Matrix<double, 2, 2> I2 = Eigen::Matrix<double, 2, 2>::Identity();

      double ddz_com = zdot_traj[i](1);
      double z_com = z_traj[i](0);
      double lambda = (ddz_com + grav) / (z_com + z_c);
      A.block<2,2>(0,2) = I2;
      A.block<2,2>(2,0) = lambda * I2;
      A.block<2,2>(2,4) = -lambda * I2;
      A.block<2,2>(4,6) = I2;
      B.block<2,2>(6,0) = I2;
      return A * x + B * u;
  };

  auto f_xy = [this, &z_traj, &zdot_traj](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> QP_XY::VectorX
  {
      Eigen::Matrix<double, QP_XY::NX, QP_XY::NX> A = Eigen::Matrix<double, QP_XY::NX, QP_XY::NX>::Zero();
      Eigen::Matrix<double, QP_XY::NX, QP_XY::NU> B = Eigen::Matrix<double, QP_XY::NX, QP_XY::NU>::Zero();
      Eigen::Matrix<double, 2, 2> I2 = Eigen::Matrix<double, 2, 2>::Identity();

      double ddz_com = zdot_traj[i](1);
      double z_com = z_traj[i](0);

      double lambda = (ddz_com + grav) / (z_com + z_c);
      A.block<2,2>(0,2) = I2;
      A.block<2,2>(2,0) = lambda * I2;
      A.block<2,2>(2,4) = -lambda * I2;
      A.block<2,2>(4,6) = I2;
      B.block<2,2>(6,0) = I2;
      return x + Δ * (A * x + B * u);
  };

  auto fx_xy = [this, &z_traj, &zdot_traj](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_XY::NX, QP_XY::NX>
  {
      Eigen::Matrix<double, QP_XY::NX, QP_XY::NX> A = Eigen::Matrix<double, QP_XY::NX, QP_XY::NX>::Zero();
      Eigen::Matrix<double, 2, 2> I2 = Eigen::Matrix<double, 2, 2>::Identity();

      double ddz_com = zdot_traj[i](1);
      double z_com = z_traj[i](0);
      double lambda = (ddz_com + grav) / (z_com + z_c);
      A.block<2,2>(0,2) = I2;
      A.block<2,2>(2,0) = lambda * I2;
      A.block<2,2>(2,4) = -lambda * I2;
      A.block<2,2>(4,6) = I2;
      return Eigen::Matrix<double,QP_XY::NX,QP_XY::NX>::Identity() + Δ * A;
  };

  auto fu_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_XY::NX, QP_XY::NU>
  {
      Eigen::Matrix<double, QP_XY::NX, QP_XY::NU> B = Eigen::Matrix<double, QP_XY::NX, QP_XY::NU>::Zero();
      Eigen::Matrix<double, 2, 2> I2 = Eigen::Matrix<double, 2, 2>::Identity();
      B.block<2,2>(6,0) = I2;
      return Δ * B;
  };


  auto L_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> Eigen::Matrix<double, 1, 1>
  {       
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_2D = Qp_xy.C_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_error_2D = pcom_2D - pcom_ref.col(i).segment<2>(0);

      Eigen::Matrix<double, QP_XY::NY, 1> vcom_2D = Qp_xy.Cv_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vcom_error_2D = vcom_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> vpc_2D = Qp_xy.Cv_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vpc_error_2D = vpc_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> pc_2D = Qp_xy.C_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pc_error_2D = pc_2D - pc_ref.col(i);
      Eigen::Matrix<double, 1, 1> ret;
      ret(0,0) = w_z * pc_error_2D.squaredNorm() 
                + w_c * pcom_error_2D.squaredNorm()
                + w_cd * vcom_error_2D.squaredNorm()
                + w_zd * vpc_error_2D.squaredNorm()
                + w_acc * u.squaredNorm();
      return ret;
  };

  auto Lx_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> QP_XY::VectorX
  { 
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_2D = Qp_xy.C_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_error_2D = pcom_2D - pcom_ref.col(i).segment<2>(0);

      Eigen::Matrix<double, QP_XY::NY, 1> vcom_2D = Qp_xy.Cv_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vcom_error_2D = vcom_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> vpc_2D = Qp_xy.Cv_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vpc_error_2D = vpc_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> pc_2D = Qp_xy.C_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pc_error_2D = pc_2D - pc_ref.col(i);
      return 2 * w_z * Qp_xy.C_pc.transpose() * pc_error_2D
             + 2 * w_c * Qp_xy.C_com.transpose() * pcom_error_2D
             + 2 * w_zd * Qp_xy.Cv_pc.transpose() * vpc_error_2D 
             + 2 * w_cd * Qp_xy.Cv_com.transpose() * vcom_error_2D;
  };

  auto Lu_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> QP_XY::VectorU
  {
      return 2 * w_acc * u;
  };

  auto Lxx_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_XY::NX, QP_XY::NX>
  {
      return 2 * w_z * Qp_xy.C_pc.transpose() * Qp_xy.C_pc 
      + 2 * w_c * Qp_xy.C_com.transpose() * Qp_xy.C_com 
      + 2 * w_zd * Qp_xy.Cv_pc.transpose() * Qp_xy.Cv_pc 
      + 2 * w_cd * Qp_xy.Cv_com.transpose() * Qp_xy.Cv_com;
  };

  auto Luu_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_XY::NU, QP_XY::NU>
  {
      return 2 * w_acc * Eigen::Matrix2d::Identity();;
  };

  auto Lux_xy = [this](const QP_XY::VectorX &x, const QP_XY::VectorU &u, const int &i) -> Eigen::Matrix<double, QP_XY::NU, QP_XY::NX>
  {
      return Eigen::Matrix<double, QP_XY::NU, QP_XY::NX>::Zero();
  };

  // ------------------------------------------------------------- //
  // terminal cost ----------------------------------------------- //

  auto L_ter_xy = [this](const QP_XY::VectorX &x) -> Eigen::Matrix<double, 1, 1>
  {
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_2D = Qp_xy.C_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_error_2D = pcom_2D - pcom_ref.col(NH).segment<2>(0);

      Eigen::Matrix<double, QP_XY::NY, 1> vcom_2D = Qp_xy.Cv_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vcom_error_2D = vcom_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> vpc_2D = Qp_xy.Cv_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vpc_error_2D = vpc_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> pc_2D = Qp_xy.C_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pc_error_2D = pc_2D - pc_ref.col(NH);
      Eigen::Matrix<double, 1, 1> ret;
      ret(0,0) = w_z * pc_error_2D.squaredNorm() 
      + w_c * pcom_error_2D.squaredNorm() 
      + w_zd * vpc_error_2D.squaredNorm() 
      + w_cd * vcom_error_2D.squaredNorm();
      return ret;
  };

  auto L_terx_xy = [this](const QP_XY::VectorX &x) -> QP_XY::VectorX
  {
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_2D = Qp_xy.C_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pcom_error_2D = pcom_2D - pcom_ref.col(NH).segment<2>(0);

      Eigen::Matrix<double, QP_XY::NY, 1> vcom_2D = Qp_xy.Cv_com * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vcom_error_2D = vcom_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> vpc_2D = Qp_xy.Cv_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> vpc_error_2D = vpc_2D;

      Eigen::Matrix<double, QP_XY::NY, 1> pc_2D = Qp_xy.C_pc * x;
      Eigen::Matrix<double, QP_XY::NY, 1> pc_error_2D = pc_2D - pc_ref.col(NH);
      return 2 * w_z * Qp_xy.C_pc.transpose() * pc_error_2D 
             + 2 * w_c * Qp_xy.C_com.transpose() * pcom_error_2D
             + 2 * w_zd * Qp_xy.Cv_pc.transpose() * vpc_error_2D 
             + 2 * w_cd * Qp_xy.Cv_com.transpose() * vcom_error_2D;
  };

  auto L_terxx_xy = [this](const QP_XY::VectorX &x) -> Eigen::Matrix<double, QP_XY::NX, QP_XY::NX>
  {
      return 2 * w_z * Qp_xy.C_pc.transpose() * Qp_xy.C_pc 
      + 2 * w_c * Qp_xy.C_com.transpose() * Qp_xy.C_com 
      + 2 * w_zd * Qp_xy.Cv_pc.transpose() * Qp_xy.Cv_pc 
      + 2 * w_cd * Qp_xy.Cv_com.transpose() * Qp_xy.Cv_com;
  };


  // ------------------------------------------------------------- //
  // terminal constraint ----------------------------------------- //

  auto h_ter = [this](const QP_XY::VectorX &x) -> Eigen::Matrix<double, QP_XY::NY, 1>
  {
      Eigen::Matrix<double, QP_XY::NY, QP_XY::NX> H = Eigen::Matrix<double, QP_XY::NY, QP_XY::NX>::Zero();
      H = Qp_xy.C_com - Qp_xy.C_pc;
      return H * x;
  };

  auto h_terx = [this](const QP_XY::VectorX &x) -> Eigen::Matrix<double, QP_XY::NY, QP_XY::NX>
  {
      Eigen::Matrix<double, QP_XY::NY, QP_XY::NX> H = Eigen::Matrix<double, QP_XY::NY, QP_XY::NX>::Zero();
      H = Qp_xy.C_com - Qp_xy.C_pc;
      return H;
  };




  // initialize problem
  auto solver_xy = DdpSolver<QP_XY::NX, QP_XY::NU, 0, QP_XY::NY, 0, NH>(
    f_xy, fx_xy, fu_xy,
    L_xy, Lx_xy, Lu_xy, Lxx_xy, Luu_xy, Lux_xy,
    L_ter_xy, L_terx_xy, L_terxx_xy,
    h_ter, h_terx,
    nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr,
    true,
    SOLVER_MAX_ITER,
    1.0,
    1e-1,
    0.5); 


  Eigen::Vector<double, QP_XY::NX> x0_xy = Eigen::Vector<double, QP_XY::NX>::Zero();
  x0_xy.segment<2>(0) = x0.segment<2>(0);
  x0_xy.segment<2>(2) = x0.segment<2>(3);
  x0_xy.segment<4>(4) = x0.segment<4>(6);


  std::array<Eigen::Vector<double, QP_XY::NX>, NH+1> xy_traj;
  xy_traj[0] = x0_xy;
  std::array<Eigen::Vector<double, QP_XY::NU>, NH  > acc_traj;
  std::array<Eigen::Vector<double, QP_XY::NX>, NH  > xydot_traj;

  // set warm-start trajectories
  std::array<Eigen::Vector<double, QP_XY::NX>, NH+1> xy_guess;
  for (int i = 0; i < NH+1; ++i)
    xy_guess[i] = x0_xy;
  std::array<Eigen::Vector<double, QP_XY::NU>, NH> acc_guess;
  for (int i = 0; i < NH; ++i)
    acc_guess[i] = Qp_xy.u_prev;

  solver_xy.set_initial_state(x0_xy);
  solver_xy.set_x_warmstart(xy_guess);
  solver_xy.set_u_warmstart(acc_guess);

  // solve DDP
  solver_xy.solve();

  // integrate dynamics
  for (int i = 0; i < NH; ++i) {
    QP_XY::VectorX x_i = xy_traj[i];
    QP_XY::VectorU u_i = solver_xy.u[i];
    xy_traj[i+1] = f_xy(x_i, u_i, i);
    xydot_traj[i] = x_dot_xy(x_i, u_i, i);
    acc_traj[i] = u_i;
  }

  // std::cout << "acc_traj " << acc_traj[0] << std::endl;

  const auto& xy_prediction = xy_traj[1];
  const auto& xydot_prediction = xydot_traj[1];
  const auto& acc_prediction = acc_traj[0];

  Qp_xy.u_prev = acc_prediction;


  // MPC output reconstruction
  pos_com_ = Eigen::Vector3d(xy_prediction(0), xy_prediction(1), z_prediction(0));
  vel_com_ = Eigen::Vector3d(xy_prediction(2), xy_prediction(3), z_prediction(1));
  acc_com_ = Eigen::Vector3d(xydot_prediction(2), xydot_prediction(3), zdot_prediction(1));
  pos_pc_ = Eigen::Vector3d(xy_prediction(4), xy_prediction(5), 0.0);
  vel_pc_ = Eigen::Vector3d(xy_prediction(6), xy_prediction(7), 0.0);
  acc_pc_ = Eigen::Vector3d(acc_prediction(0), acc_prediction(1), 0.0);


  std::array<Eigen::Vector<double, N_STATE>, NH+1> x_traj;
  x_traj[0] = x0;
  std::array<Eigen::Vector<double, N_OUTPUT>, NH  > u_traj;

  for (int i = 0; i < NH; ++i) {
    x_traj[i+1].segment<2>(0) = xy_traj[i+1].segment<2>(0);
    x_traj[i+1](2) = z_traj[i+1](0);
    x_traj[i+1].segment<2>(3) = xy_traj[i+1].segment<2>(2);
    x_traj[i+1](5) = z_traj[i+1](1);
    x_traj[i+1].segment<2>(6) = xy_traj[i+1].segment<2>(4);
    x_traj[i+1].segment<2>(8) = xy_traj[i+1].segment<2>(6);

    u_traj[i].segment<2>(0) = acc_traj[i];
    u_traj[i](2) = fz_traj[i](0);
  }


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












