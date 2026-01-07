#include <MPC.hpp>


#include <chrono>


double wrapToPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

double unwrapNear(double theta_wrapped, double theta_prev) {
  return theta_prev + wrapToPi(theta_wrapped - theta_prev);
}

Eigen::Vector<double, labrob::MPC::NX> labrob::MPC::get_DFIP_state(Eigen::Vector<double, N_IN> x_IN,
                                                                   bool set_d = false){
  Eigen::Vector3d pcom       = x_IN.segment<3>(0);
  Eigen::Vector3d vcom       = x_IN.segment<3>(3);
  Eigen::Vector3d pl_world   = x_IN.segment<3>(6);
  Eigen::Vector3d pr_world   = x_IN.segment<3>(9);
  Eigen::Vector3d dpl_world  = x_IN.segment<3>(12);
  Eigen::Vector3d dpr_world  = x_IN.segment<3>(15);

  Eigen::Vector3d c_world   = (pl_world + pr_world) / 2;
  Eigen::Vector3d vc_world  = (dpl_world + dpr_world) / 2;

  // extract theta
  Eigen::Vector3d diff = pl_world - pr_world;
  double theta_wrapped = atan2(-diff.x(), diff.y());
  theta_prev_ = unwrapNear(theta_wrapped, theta_prev_);
  double theta = theta_prev_;

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R << cos(theta), -sin(theta), 0,
  sin(theta), cos(theta),  0,
  0, 0, 1;
  
  if(set_d){
    // body-frame positions:
    Eigen::Vector3d pl_body = R.transpose() * pl_world;
    Eigen::Vector3d pr_body = R.transpose() * pr_world;
    // unicycle offset
    d = (pl_body - pr_body).norm();
  }

  // body-frame velocities:
  Eigen::Vector3d dpl_body = R.transpose() * dpl_world;
  Eigen::Vector3d dpr_body = R.transpose() * dpr_world;

  // differential drive inputs
  double w = (dpr_body.x() - dpl_body.x()) / d;
  double v = (dpr_body.x() + dpl_body.x()) / 2;

  Eigen::Vector<double, NX> x0 = Eigen::Vector<double, NX>::Zero();
  x0.segment<3>(0) = pcom;
  x0.segment<3>(3) = vcom;
  x0.segment<3>(6) = c_world;
  x0(9)  = vc_world.z();
  x0(10) = theta;
  x0(11)  = v;
  x0(12)  = w;
  return x0;
}

void labrob::MPC::init_solver(Eigen::Vector<double, N_IN> x_IN){

  auto x0 = get_DFIP_state(x_IN, true);

  // stack running models
  std::vector<std::shared_ptr<ActionModelAbstract>> runningModels;
  runningModels.reserve(NH);
  di_models_.reserve(NH);

  for (int i = 0; i < NH; ++i) {
    auto model = std::make_shared<DFIPActionModel>(NX, NU, Δ, d, m);
    di_models_.push_back(model);
    runningModels.push_back(model);
  }
  
  // terminal model 
  terminalModel_ = std::make_shared<DFIPActionModel>(NX, 0, Δ, d, m);
  
  problemPtr_ = std::make_shared<ShootingProblem>(x0, runningModels, terminalModel_);
  
  // Initialize the FDDP solver
  solver = std::make_shared<SolverFDDP>(problemPtr_);

  // Initialize guess trajectory
  xs.resize(NH + 1, x0);
  us.resize(NH, Eigen::VectorXd::Zero(NU));

  Eigen::VectorXd fl0 = Eigen::Vector3d::Zero();
  Eigen::VectorXd fr0 = Eigen::Vector3d::Zero();
  fl0(2) = m*grav/2;
  fr0(2) = m*grav/2;
  for (int i =0; i < NH; ++i ){
    us[i].segment<3>(3) = fl0;
    us[i].segment<3>(6) = fr0;
  }
}

void labrob::MPC::solve(Eigen::Vector<double, N_IN> x_IN){

  // update reference in the Action models
  update_actionModel();

  // set x0
  auto x0 = get_DFIP_state(x_IN);
  problemPtr_->set_x0(x0);

  auto t0 = std::chrono::high_resolution_clock::now();

  // solve the problem
  xs[0] = x0;   // to improve feasibility
  solver->solve(xs, us, SOLVER_MAX_ITER);

  auto t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms = t1 - t0;
  std::cout << "Solve time: " << ms.count() << " ms\n"<< std::endl;




  // get solution
  auto x_traj = solver->get_xs();
  auto u_traj = solver->get_us();

  // Shift by one guess trajectory
  for (unsigned int i = 0; i < NH; ++i)
      xs[i] = x_traj[i + 1];  
  xs[NH] = x_traj[NH]; 

  for (unsigned int i = 0; i < NH - 1; ++i)
      us[i] = u_traj[i + 1];
  us[NH - 1] = u_traj[NH- 1]; 



  // Build solution
  const auto& u_prediction = u_traj[0];

  // inputs 
  double a            = u_prediction(0);
  double ac_z         = u_prediction(1);
  double alpha        = u_prediction(2);
  Eigen::Vector3d fcl = u_prediction.segment<3>(3);
  Eigen::Vector3d fcr = u_prediction.segment<3>(6);

  // current state
  Eigen::Vector3d pcom_curr = x_IN.segment<3>(0);
  Eigen::Vector3d vcom_curr = x_IN.segment<3>(3);
  Eigen::Vector3d pl_curr   = x_IN.segment<3>(6);
  Eigen::Vector3d pr_curr   = x_IN.segment<3>(9);
  Eigen::Vector3d dpl_curr  = x_IN.segment<3>(12);
  Eigen::Vector3d dpr_curr  = x_IN.segment<3>(15);

  double theta_curr = x0(10);
  double v_curr     = x0(11);
  double w_curr     = x0(12);

  Eigen::Vector3d g_vec = Eigen::Vector3d(0,0,-grav);

  Eigen::Vector3d vector_off = Eigen::Vector3d(0.0, d/2, 0.0);
  Eigen::Matrix3d dR_curr = Eigen::Matrix3d::Zero();
  dR_curr << -sin(theta_curr), -cos(theta_curr), 0,
        cos(theta_curr), -sin(theta_curr),  0,
        0, 0, 0;

  Eigen::Matrix3d ddR_curr = Eigen::Matrix3d::Zero();
  ddR_curr << -cos(theta_curr), sin(theta_curr), 0,
        -sin(theta_curr), -cos(theta_curr),  0,
        0, 0, 0;

  Eigen::Vector3d ddc;
  ddc(0) = a * cos(theta_curr) - v_curr * sin(theta_curr) * w_curr;
  ddc(1) = a * sin(theta_curr) + v_curr * cos(theta_curr) * w_curr;
  ddc(2) = ac_z;

  // integrate inputs
  acc_com_ = 1/m * (fcl + fcr) + g_vec;
  vel_com_ = vcom_curr + dt_ * acc_com_;
  pos_com_ = pcom_curr + dt_ * vel_com_;
  
  acc_pl_  = ddc + (ddR_curr * w_curr * w_curr + dR_curr * alpha) * vector_off;
  vel_pl_  = dpl_curr + dt_ * acc_pl_;
  pos_pl_  = pl_curr + dt_ * vel_pl_;
  
  acc_pr_  = ddc - (ddR_curr * w_curr * w_curr + dR_curr * alpha) * vector_off;
  vel_pr_  = dpr_curr + dt_ * acc_pr_;
  pos_pr_  = pr_curr + dt_ * vel_pr_;
  
  alpha_   = alpha;
  omega_   = w_curr + dt_ * alpha_;
  theta_   = theta_curr +  dt_ * omega_;




  if(record_logs){
    // create folder if it does not exist
    std::string folder = "/tmp/mpc_data/" + std::to_string(t_msec);
    std::string command = "mkdir -p " + folder;
    int _ = system(command.c_str());

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







void labrob::MPC::update_actionModel(){
  const double dt_ms = Δ * 1000.0;     // Delta in seconds
  const double t0_ms = t_msec;         // current time in ms

  // running stages
  for (int i = 0; i < NH; ++i) {
      double t_prevision = t_msec + dt_ms * i;
      di_models_[i]->setReference(walkingPlanner_ptr_->get_xref_at_time_ms(t_prevision),
                                  walkingPlanner_ptr_->get_uref_at_time_ms(t_prevision));
  }
    
  // terminal stage
  double t_prevision = t_msec + dt_ms * NH;
  terminalModel_->setReference(walkingPlanner_ptr_->get_xref_at_time_ms(t_prevision));   
}