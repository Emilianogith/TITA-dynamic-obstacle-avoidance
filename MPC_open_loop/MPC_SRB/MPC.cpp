#include <MPC.hpp>
#include <chrono>

void labrob::MPC::init_solver(Eigen::Vector<double, NX> x0){

  I << 0.061855, 0.0, 0.0,
       0.0, 0.036934, 0.0,
       0.0, 0.0, 0.071232;
  I_inv_ = I.inverse();

  // stack running models
  std::vector<std::shared_ptr<ActionModelAbstract>> runningModels;
  runningModels.reserve(NH);
  di_models_.reserve(NH);

  for (int i = 0; i < NH; ++i) {
    auto model = std::make_shared<DFIPActionModel>(NX, NU, Δ, d, m, I);
    di_models_.push_back(model);
    runningModels.push_back(model);
  }
  
  // terminal model 
  terminalModel_ = std::make_shared<DFIPActionModel>(NX, 0, Δ, d, m, I);
  
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
    us[i].segment<3>(4) = fl0;
    us[i].segment<3>(7) = fr0;
  }

  // resize log metrices
  X.resize(NH, NX);
  U.resize(NH - 1, NU);
}

void labrob::MPC::solve(Eigen::Vector<double, NX> x0){

  // update reference in the Action models
  update_actionModel();

  // set x0
  problemPtr_->set_x0(x0);

  // auto t0 = std::chrono::high_resolution_clock::now();

  // solve the problem
  xs[0] = x0;   // to improve feasibility
  solver->solve(xs, us, SOLVER_MAX_ITER);

  // auto t1 = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double, std::milli> ms = t1 - t0;
  // std::cout << "Solve time: " << ms.count() << " ms\n"<< std::endl;




  // get solution
  auto x_traj = solver->get_xs();
  auto u_traj = solver->get_us();


  // Shift by one guess trajectory
  for (unsigned int i = 0; i < NH; ++i)
  {
      xs[i] = x_traj[i + 1];  
      X.row(i) = x_traj[i].transpose();    // save for logging
  }
  xs[NH] = x_traj[NH]; 

  for (unsigned int i = 0; i < NH - 1; ++i)
  {
      us[i] = u_traj[i + 1];
      U.row(i) = u_traj[i].transpose();    // save for logging
  }
  us[NH - 1] = u_traj[NH- 1]; 



  // Build solution
  const auto& u_prediction = u_traj[0];

  Eigen::Vector3d g_vec = Eigen::Vector3d(0,0,-grav);

  // inputs 
  double ar           = u_prediction(0);
  double ar_z         = u_prediction(1);
  double al           = u_prediction(2);
  double al_z         = u_prediction(3);
  Eigen::Vector3d fr = u_prediction.segment<3>(4);
  Eigen::Vector3d fl = u_prediction.segment<3>(7);

  // current state
  Eigen::Vector3d pcom_curr    = x0.segment<3>(0);
  Eigen::Vector3d vcom_curr    = x0.segment<3>(3);
  Eigen::Vector3d rpy_curr     = x0.segment<3>(6);
  Eigen::Vector3d ang_vel_curr = x0.segment<3>(9);
  Eigen::Vector3d pr_curr      = x0.segment<3>(12);
  double vr_curr               = x0(15);
  double vr_z_curr             = x0(16);
  Eigen::Vector3d pl_curr      = x0.segment<3>(17);
  double vl_curr               = x0(20);
  double vl_z_curr             = x0(21);

  Eigen::Vector3d pr_vel_curr;
  pr_vel_curr(0)  = vr_curr * cos(rpy_curr(2));
  pr_vel_curr(1)  = vr_curr * sin(rpy_curr(2));
  pr_vel_curr(2)  = vr_z_curr;

  Eigen::Vector3d pl_vel_curr;
  pl_vel_curr(0)  = vl_curr * cos(rpy_curr(2));
  pl_vel_curr(1)  = vl_curr * sin(rpy_curr(2));
  pl_vel_curr(2)  = vl_z_curr;


  // integrate inputs
  acc_com_ = 1/m * (fl + fr) + g_vec;
  vel_com_ = vcom_curr + dt_ * acc_com_;
  pos_com_ = pcom_curr + dt_ * vcom_curr;

  acc_rpy_ =  I_inv_ * (((pl_curr - pcom_curr).cross(fl) + (pr_curr - pcom_curr).cross(fr)) - ang_vel_curr.cross(I*ang_vel_curr));
  vel_rpy_ = ang_vel_curr + dt_ * acc_rpy_;
  pos_rpy_ = rpy_curr + dt_ * ang_vel_curr;
  
  acc_pr_(0)  = ar * cos(rpy_curr(2)) - vr_curr * sin(rpy_curr(2)) * ang_vel_curr(2);
  acc_pr_(1)  = ar * sin(rpy_curr(2)) + vr_curr * cos(rpy_curr(2)) * ang_vel_curr(2);
  acc_pr_(2)  = ar_z;

  double vr_next = vr_curr + dt_ * ar;

  vel_pr_(0)  = vr_next * cos(rpy_curr(2));
  vel_pr_(1)  = vr_next * sin(rpy_curr(2));
  vel_pr_(2)  = vr_z_curr + dt_ * ar_z;

  pos_pr_  = pr_curr + dt_ * pr_vel_curr;
  
  acc_pl_(0)  = al * cos(rpy_curr(2)) - vl_curr * sin(rpy_curr(2)) * ang_vel_curr(2);
  acc_pl_(1)  = al * sin(rpy_curr(2)) + vl_curr * cos(rpy_curr(2)) * ang_vel_curr(2);
  acc_pl_(2)  = al_z;

  double vl_next = vl_curr + dt_ * al;

  vel_pl_(0)  = vl_next * cos(rpy_curr(2));
  vel_pl_(1)  = vl_next * sin(rpy_curr(2));
  vel_pl_(2)  = vl_z_curr + dt_ * al_z;

  pos_pl_  = pl_curr + dt_ * pl_vel_curr;
   
}







void labrob::MPC::update_actionModel(){
  const double dt_ms = Δ * 1000.0;     // Delta in seconds

  // running stages
  for (int i = 0; i < NH; ++i) {
      double t_prevision = t_msec_ + dt_ms * i;
      di_models_[i]->setReference(walkingPlanner_ptr_->get_xref_at_time_ms(t_prevision),
                                  walkingPlanner_ptr_->get_uref_at_time_ms(t_prevision));
  }
    
  // terminal stage
  double t_prevision = t_msec_ + dt_ms * NH;
  terminalModel_->setReference(walkingPlanner_ptr_->get_xref_at_time_ms(t_prevision));   
}