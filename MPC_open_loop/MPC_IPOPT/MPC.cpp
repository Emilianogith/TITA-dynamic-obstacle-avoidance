#include <MPC.hpp>

#include <chrono>


void labrob::MPC::solve(Eigen::Vector<double, NX> x0){

  auto t_start = std::chrono::high_resolution_clock::now();

  casadi::DM p0 = casadi::DM::zeros(12,1);
  p0(0) = x0(0);
  p0(1) = x0(1);
  p0(2) = x0(2);
  p0(3) = x0(3);
  p0(4) = x0(4);
  p0(5) = x0(5);
  p0(6) = x0(6);
  p0(7) = x0(7);
  p0(8) = x0(8);
  p0(9) = x0(9);
  p0(10) = x0(10);
  p0(11) = x0(11);

  // set initial conditions
  opti_ptr_->set_value(p_x0, p0);
  
  
  // Solve
  auto sol = opti_ptr_->solve();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
  std::cout << "Solver time: " << duration.count() << " ms" << std::endl;

  // update initial guess for the next iteration
  set_warm_start(sol, p0);



  // Build predicted trajectory for saving logs
  std::array<Eigen::Matrix<double,NX,1>, NH+1> x_traj;
  x_traj[0] = x0;
  std::array<Eigen::Matrix<double,NU,1>,  NH>   u_traj;
  std::array<Eigen::Matrix<double,NX,1>, NH+1> xdot_traj;

  int base_idx_u = 2;

  for (int k = 0; k < NH; ++k) {

    Eigen::Matrix<double,NU,1> uk;
    casadi::DM ac_DM = sol.value(input_vars_[base_idx_u*k + 0]);
    casadi::DM fc_DM = sol.value(input_vars_[base_idx_u*k + 1]);
    
    for (int i = 0; i < 3; ++i){
      uk(i)   = double(ac_DM(i));
      uk(i+3) = double(fc_DM(i));
    }
    u_traj[k] = uk;

    Eigen::Vector3d ac = uk.segment<3>(0);
    Eigen::Vector3d fc = uk.segment<3>(3);
    
    VectorX xk = VectorX::Zero();
    
    Eigen::Vector3d pcom = x_traj[k].segment<3>(0);
    Eigen::Vector3d vcom = x_traj[k].segment<3>(3);
    Eigen::Vector3d pc = x_traj[k].segment<3>(6);
    Eigen::Vector3d vc = x_traj[k].segment<3>(9);

    Eigen::Vector3d grav_vec = Eigen::Vector3d(0,0,-grav);

    xk.segment<3>(0) = pcom + Δ * vcom;
    xk.segment<3>(3) = vcom + Δ * (fc / m + grav_vec);
    xk.segment<3>(6) = pc + Δ * vc;
    xk.segment<3>(9) = vc + Δ * ac;
    x_traj[k+1] = xk;

    VectorX xdotk = VectorX::Zero();
    xdotk.segment<3>(0) = vcom;
    xdotk.segment<3>(3) = fc / m + grav_vec;
    xdotk.segment<3>(6) = vc;
    xdotk.segment<3>(9) = ac;
    xdot_traj[k] = xdotk;
  }


  const auto x_prediction = x_traj[1];
  const auto xdot_prediction = xdot_traj[0];
  const auto u_prediction = u_traj[0];


  pos_com_ = Eigen::Vector3d(x_prediction(0), x_prediction(1), x_prediction(2));
  vel_com_ = Eigen::Vector3d(x_prediction(3), x_prediction(4), x_prediction(5));
  acc_com_ = Eigen::Vector3d(xdot_prediction(3), xdot_prediction(4), xdot_prediction(5));
  pos_zmp_ = Eigen::Vector3d(x_prediction(6), x_prediction(7), x_prediction(8));
  vel_zmp_ = Eigen::Vector3d(x_prediction(9), x_prediction(10), x_prediction(11));
  acc_zmp_ = Eigen::Vector3d(u_prediction(0), u_prediction(1), u_prediction(2));





  


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



void labrob::MPC::set_warm_start(const casadi::OptiSol& sol, const casadi::DM& p0){

  // set x_0 explicitly from parameter p_x0
  opti_ptr_->set_initial(state_vars_[0], p0(casadi::Slice(0,3)));
  opti_ptr_->set_initial(state_vars_[1], p0(casadi::Slice(3,6)));
  opti_ptr_->set_initial(state_vars_[2], p0(casadi::Slice(6,9)));
  opti_ptr_->set_initial(state_vars_[3], p0(casadi::Slice(9,12)));

  // SHIFT STATES
  for (int k = 1; k < NH; k++) {
      // next state's pcom, vcom, pc, vc
      casadi::DM p_next = sol.value(state_vars_[4*(k+1) + 0]);
      casadi::DM v_next = sol.value(state_vars_[4*(k+1) + 1]);
      casadi::DM pc_next = sol.value(state_vars_[4*(k+1) + 2]);
      casadi::DM vc_next = sol.value(state_vars_[4*(k+1) + 3]);

      opti_ptr_->set_initial(state_vars_[4*k + 0], p_next);
      opti_ptr_->set_initial(state_vars_[4*k + 1], v_next);
      opti_ptr_->set_initial(state_vars_[4*k + 2], pc_next);
      opti_ptr_->set_initial(state_vars_[4*k + 3], vc_next);
  }

  // repeat last state
  opti_ptr_->set_initial(state_vars_[4*NH + 0], sol.value(state_vars_[4*NH + 0]));
  opti_ptr_->set_initial(state_vars_[4*NH + 1], sol.value(state_vars_[4*NH + 1]));
  opti_ptr_->set_initial(state_vars_[4*NH + 2], sol.value(state_vars_[4*NH + 2]));
  opti_ptr_->set_initial(state_vars_[4*NH + 3], sol.value(state_vars_[4*NH + 3]));

  // SHIFT INPUTS
  for (int k = 0; k < NH - 1; k++) {
      casadi::DM ac_next = sol.value(input_vars_[2*(k+1) + 0]);
      casadi::DM fc_next = sol.value(input_vars_[2*(k+1) + 1]);

      opti_ptr_->set_initial(input_vars_[2*k + 0], ac_next);
      opti_ptr_->set_initial(input_vars_[2*k + 1], fc_next);
  }

  // repeat last input
  opti_ptr_->set_initial(input_vars_[2*(NH-1) + 0], sol.value(input_vars_[2*(NH-1) + 0]));
  opti_ptr_->set_initial(input_vars_[2*(NH-1) + 1], sol.value(input_vars_[2*(NH-1) + 1]));

}













// OLD TRAJECTORY CONSTRUCTION




//   // Build predicted trajectory for saving logs
//   std::array<Eigen::Matrix<double,NX,1>, NH+1> x_traj;
//   x_traj.fill(VectorX::Zero());
//   std::array<Eigen::Matrix<double,NU,1>,  NH>   u_traj;

//   int base_idx_x = 4;
//   int base_idx_u = 2;

//   for (int k = 0; k <= NH; ++k) {

//     // Eigen::Matrix<double,NX,1> xk;
//     // casadi::DM pcom = sol.value(state_vars_[base_idx_x*k + 0]);
//     // casadi::DM vcom = sol.value(state_vars_[base_idx_x*k + 1]);
//     // casadi::DM pc   = sol.value(state_vars_[base_idx_x*k + 2]);
//     // casadi::DM vc   = sol.value(state_vars_[base_idx_x*k + 3]);

//     // for (int i = 0; i < 3; ++i){
//     //     xk(i)      = double(pcom(i));
//     //     xk(i+3)    = double(vcom(i));
//     //     xk(i+6)    = double(pc(i));
//     //     xk(i+9)    = double(vc(i));
//     // }

//     // x_traj[k] = xk;

//     if (k < NH){
//       Eigen::Matrix<double,NU,1> uk;
//       casadi::DM ac = sol.value(input_vars_[base_idx_u*k + 0]);
//       casadi::DM fc = sol.value(input_vars_[base_idx_u*k + 1]);

//       for (int i = 0; i < 3; ++i){
//           uk(i)   = double(ac(i));
//           uk(i+3) = double(fc(i));
//       }

//       u_traj[k] = uk;
      
//   }
// }


//   // assign first prediction
//   int prediction_idx_x = 4; // to skip first state 

//   casadi::DM ac = sol.value(input_vars_[0]);
//   casadi::DM fc = sol.value(input_vars_[1]);

//   casadi::DM pcom = sol.value(state_vars_[prediction_idx_x + 0]);
//   casadi::DM vcom = sol.value(state_vars_[prediction_idx_x + 1]);
//   casadi::DM acom = fc / m + g;
  
//   casadi::DM pc = sol.value(state_vars_[prediction_idx_x + 2]);
//   casadi::DM vc = sol.value(state_vars_[prediction_idx_x + 3]);


//   for (int i = 0; i < 3; ++i){
//       pos_com_(i) = static_cast<double>(pcom(i));
//       vel_com_(i) = static_cast<double>(vcom(i));
//       acc_com_(i) = static_cast<double>(acom(i));
//       pos_zmp_(i) = static_cast<double>(pc(i));
//       vel_zmp_(i) = static_cast<double>(vc(i));
//       acc_zmp_(i) = static_cast<double>(ac(i));
//   }