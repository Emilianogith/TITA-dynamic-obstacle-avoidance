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

  std::cout << "u_prev: " << u_prev << std::endl;


  
  // set initial conditions
  opti_ptr_->set_value(p_x0, p0);
  
  std::cout << "x0 = " << x0 << std::endl;
  std::cout << "p_x0 = " << opti_ptr_->value(p_x0) << std::endl;




  // set initial guess
  // for (int k = 0; k <= NH; ++k) {
  //   int base_x = 4 * k; 

  //   casadi::MX pcom = state_vars_[base_x + 0];
  //   casadi::MX vcom = state_vars_[base_x + 1];
  //   casadi::MX pc   = state_vars_[base_x + 2];
  //   casadi::MX vc   = state_vars_[base_x + 3];
    
  //   opti_ptr_->set_initial(pcom, p0(casadi::Slice(0,3)));
  //   opti_ptr_->set_initial(vcom, p0(casadi::Slice(3,6)));
  //   opti_ptr_->set_initial(pc,   p0(casadi::Slice(6,9)));
  //   opti_ptr_->set_initial(vc,   p0(casadi::Slice(9,12)));

  //   if (k < NH){
  //     int base_u = 2 * k;  
  //     auto ac   = input_vars_[base_u + 0];
  //     auto fc   = input_vars_[base_u + 1];
  //     opti_ptr_->set_initial(fc,   casadi::DM({u_prev(0), u_prev(1), u_prev(2)}));
  //     opti_ptr_->set_initial(ac,   casadi::DM({u_prev(3), u_prev(4), u_prev(5)}));
  //   }
  // }



  // for (int k = 0; k <= NH; ++k) {
  //   int base_x = 4 * k; 

  //   casadi::MX pcom = state_vars_[base_x + 0];
  //   casadi::MX vcom = state_vars_[base_x + 1];
  //   casadi::MX pc   = state_vars_[base_x + 2];
  //   casadi::MX vc   = state_vars_[base_x + 3];

  //   casadi::DM Xk = casadi::DM::zeros(NX,1);
  //   if (k == 0){
  //     Xk = p0;
  //   }else if (k < NH){
  //     for (int i = 0; i < NX; ++i)
  //         Xk(i) = X_prev[k+1](i);
  //   }else if (k == NH){
  //     for (int i = 0; i < NX; ++i)
  //           Xk(i) = X_prev[NH](i);
  //   }


  //   opti_ptr_->set_initial(pcom, Xk(casadi::Slice(0,3)));
  //   opti_ptr_->set_initial(vcom, Xk(casadi::Slice(3,6)));
  //   opti_ptr_->set_initial(pc,   Xk(casadi::Slice(6,9)));
  //   opti_ptr_->set_initial(vc,   Xk(casadi::Slice(9,12)));

  //   if (k < NH){
  //     int base_u = 2 * k;  
  //     auto ac   = input_vars_[base_u + 0];
  //     auto fc   = input_vars_[base_u + 1];

  //     casadi::DM Uk = casadi::DM::zeros(NU,1);
  //     for (int i = 0; i < NU; ++i)
  //         Uk(i) = U_prev[k](i);


  //     opti_ptr_->set_initial(fc,   Uk(casadi::Slice(0,3)));
  //     opti_ptr_->set_initial(ac,   Uk(casadi::Slice(3,6)));
  //   }
  // }


  // warm start non funziona ??? invece senza funziona
  // set_warm_start(p0);

  // auto init_val = opti_ptr_->initial();
  // std::cout << "initial: " << init_val << std::endl;

  opti_ptr_->set_initial(opti_ptr_->x(), x_last);

  // Solve
  auto sol = opti_ptr_->solve();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
  std::cout << "Solver time: " << duration.count() << " ms" << std::endl;


  auto fc_sol   = sol.value(input_vars_[0]);
  std::cout << "fc0 = " << opti_ptr_->debug().value(fc_sol) << std::endl;




  // Check cross product constraint
  // for (int k = 0; k <= NH; ++k) {

  //     int base_x = 4 * k;   

  //     auto pc_sol   = sol.value(state_vars_[base_x + 0]);
  //     auto vc_sol   = sol.value(state_vars_[base_x + 1]);
  //     auto pcom_sol = sol.value(state_vars_[base_x + 2]);
  //     auto vcom_sol = sol.value(state_vars_[base_x + 3]);

  //     std::cout << "--- k = " << k << " ---" << std::endl;
  //     std::cout << "pc   = " << pc_sol   << std::endl;
  //     std::cout << "vc   = " << vc_sol   << std::endl;
  //     std::cout << "pcom = " << pcom_sol << std::endl;
  //     std::cout << "vcom = " << vcom_sol << std::endl;

  //     if (k < NH){
  //       int base_u = 2 * k;
  //       auto fc_sol   = sol.value(input_vars_[base_u + 0]);
  //       auto ac_sol   = sol.value(input_vars_[base_u + 1]);
  //       std::cout << "fc   = " << fc_sol   << std::endl;
  //       std::cout << "ac   = " << ac_sol   << std::endl;
    
  //     // Convert to Eigen
  //     Eigen::Vector3d pc_e, pcom_e, fc_e;
  //     for (int i = 0; i < 3; i++) {
  //         pc_e(i)   = static_cast<double>(pc_sol(i));
  //         pcom_e(i) = static_cast<double>(pcom_sol(i));
  //         fc_e(i)   = static_cast<double>(fc_sol(i));
  //     }

  //     Eigen::Vector3d cross_check = (pc_e - pcom_e).cross(fc_e);

  //     // std::cout << "Cross product check = " << cross_check.transpose() << std::endl;

  //     std::cout << "Cross product check = " << fc_e(0) * (pc_e(2) - pcom_e(2)) - fc_e(2) * (pc_e(0) - pcom_e(0)) << std::endl;

      
  //   }
  // }

  // Build predicted trajectory for saving logs
  std::array<Eigen::Matrix<double,NX,1>, NH+1> x_traj; 
  std::array<Eigen::Matrix<double,NU,1>,  NH>   u_traj;

  int base_idx_x = 4;
  int base_idx_u = 2;

  for (int k = 0; k <= NH; ++k) {

    Eigen::Matrix<double,NX,1> xk;
    casadi::DM pcom = sol.value(state_vars_[base_idx_x*k + 0]);
    casadi::DM vcom = sol.value(state_vars_[base_idx_x*k + 1]);
    casadi::DM pc   = sol.value(state_vars_[base_idx_x*k + 2]);
    casadi::DM vc   = sol.value(state_vars_[base_idx_x*k + 3]);

    for (int i = 0; i < 3; ++i){
        xk(i)      = double(pcom(i));
        xk(i+3)    = double(vcom(i));
        xk(i+6)    = double(pc(i));
        xk(i+9)    = double(vc(i));
    }

    x_traj[k] = xk;

    if (k < NH){
      Eigen::Matrix<double,NU,1> uk;
      casadi::DM ac = sol.value(input_vars_[base_idx_u*k + 0]);
      casadi::DM fc = sol.value(input_vars_[base_idx_u*k + 1]);

      for (int i = 0; i < 3; ++i){
          uk(i)   = double(ac(i));
          uk(i+3) = double(fc(i));
      }

      u_traj[k] = uk;
      
  }
}


  // assign first prediction
  // int prediction_idx_x = 4; // to skip first state 

  // casadi::DM ac = sol.value(input_vars_[0]);
  // casadi::DM fc = sol.value(input_vars_[1]);

  // casadi::DM pcom = sol.value(state_vars_[prediction_idx_x + 0]);
  // casadi::DM vcom = sol.value(state_vars_[prediction_idx_x + 1]);
  // casadi::DM acom = fc / m + g;
  
  // casadi::DM pc = sol.value(state_vars_[prediction_idx_x + 2]);
  // casadi::DM vc = sol.value(state_vars_[prediction_idx_x + 3]);


  casadi::DM ac = sol.value(input_vars_[0]);
  casadi::DM fc = sol.value(input_vars_[1]);

  casadi::DM pcom = casadi::DM::zeros(3);
  casadi::DM vcom = casadi::DM::zeros(3);
  casadi::DM acom = fc / m + g;
  
  casadi::DM pc = casadi::DM::zeros(3);
  casadi::DM vc = casadi::DM::zeros(3);

  // integrated dynamics
  pcom ==  + Δ * vcom[k];
  vcom ==  + Δ * (fc[k] / m + g);
  pc == pc[k] + Δ * vc[k];
  vc == vc[k] + Δ * ac[k];

  std::cout << "pc   = " << pc   << std::endl;
  std::cout << "vc   = " << vc   << std::endl;
  std::cout << "pcom = " << pcom << std::endl;
  std::cout << "vcom = " << vcom << std::endl;
  std::cout << "fc = " << fc << std::endl;
  std::cout << "ac = " << ac << std::endl;


  for (int i = 0; i < 3; ++i){
      pos_com_(i) = static_cast<double>(pcom(i));
      vel_com_(i) = static_cast<double>(vcom(i));
      acc_com_(i) = static_cast<double>(acom(i));
      pos_zmp_(i) = static_cast<double>(pc(i));
      vel_zmp_(i) = static_cast<double>(vc(i));
      acc_zmp_(i) = static_cast<double>(ac(i));
  }

  u_prev = u_traj[0];
  X_prev = x_traj;
  U_prev = u_traj;
  

  x_last     = sol.value(opti_ptr_->x());



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















void labrob::MPC::set_warm_start(casadi::DM p0){

  // // set guess for x0
  // casadi::MX pcom0 = state_vars_[0];
  // casadi::MX vcom0 = state_vars_[1];
  // casadi::MX pc0   = state_vars_[2];
  // casadi::MX vc0   = state_vars_[3];

  // opti_ptr_->set_initial(pcom0, p0(casadi::Slice(0,3)));
  // opti_ptr_->set_initial(vcom0, p0(casadi::Slice(3,6)));
  // opti_ptr_->set_initial(pc0,   p0(casadi::Slice(6,9)));
  // opti_ptr_->set_initial(vc0,   p0(casadi::Slice(9,12)));

  // // set guess for u0
  // casadi::MX ac   = input_vars_[0];
  // casadi::MX fc   = input_vars_[1];
  // casadi::DM Uk = casadi::DM::zeros(NU,1);
  // for (int i = 0; i < NU; ++i)
  //       Uk(i) = U_prev[1](i);
  // opti_ptr_->set_initial(ac,   Uk(casadi::Slice(0,3)));
  // opti_ptr_->set_initial(fc,   Uk(casadi::Slice(3,6)));


  // // set initial guess for k < NH
  // for (int k = 1; k < NH; ++k) {
  //   int base_x = 4 * k; 

  //   // set x initial guess
  //   casadi::MX pcom = state_vars_[base_x + 0];
  //   casadi::MX vcom = state_vars_[base_x + 1];
  //   casadi::MX pc   = state_vars_[base_x + 2];
  //   casadi::MX vc   = state_vars_[base_x + 3];

  //   casadi::DM Xk = casadi::DM::zeros(NX,1);
  //   for (int i = 0; i < NX; ++i)
  //       Xk(i) = X_prev[k](i);

  //   opti_ptr_->set_initial(pcom, Xk(casadi::Slice(0,3)));
  //   opti_ptr_->set_initial(vcom, Xk(casadi::Slice(3,6)));
  //   opti_ptr_->set_initial(pc,   Xk(casadi::Slice(6,9)));
  //   opti_ptr_->set_initial(vc,   Xk(casadi::Slice(9,12)));

  //   // set u initial guess
  //   int base_u = 2 * k;  
  //   auto ac   = input_vars_[base_u + 0];
  //   auto fc   = input_vars_[base_u + 1];
    
  //   casadi::DM Uk = casadi::DM::zeros(NU,1);
  //   // if (k == NH-1) {
  //   //    for (int i = 0; i < NU; ++i)
  //   //     Uk(i) = U_prev[k](i);
  //   // } else {
  //   //   for (int i = 0; i < NU; ++i)
  //   //     Uk(i) = U_prev[k+1](i);
  //   // }

     
  //   for (int i = 0; i < NU; ++i)
  //     Uk(i) = U_prev[k](i);
    
  //   opti_ptr_->set_initial(ac,   Uk(casadi::Slice(0,3)));
  //   opti_ptr_->set_initial(fc,   Uk(casadi::Slice(3,6)));
  // }

  // // set x NH initial guess
  // casadi::MX pcomNH = state_vars_[4*NH + 0];
  // casadi::MX vcomNH = state_vars_[4*NH + 1];
  // casadi::MX pcNH   = state_vars_[4*NH + 2];
  // casadi::MX vcNH   = state_vars_[4*NH + 3];

  // casadi::DM Xk = casadi::DM::zeros(NX,1);
  //   for (int i = 0; i < NX; ++i)
  //       Xk(i) = X_prev[NH](i);

  // opti_ptr_->set_initial(pcomNH, Xk(casadi::Slice(0,3)));
  // opti_ptr_->set_initial(vcomNH, Xk(casadi::Slice(3,6)));
  // opti_ptr_->set_initial(pcNH,   Xk(casadi::Slice(6,9)));
  // opti_ptr_->set_initial(vcNH,   Xk(casadi::Slice(9,12)));



  // auto init_val = opti_ptr_->initial();
  // std::cout << "initial: " << init_val << std::endl;



    for (int k = 0; k <= NH; ++k) {
    int base_x = 4 * k;  

    // set x initial guess
    casadi::MX pcom = state_vars_[base_x + 0];
    casadi::MX vcom = state_vars_[base_x + 1];
    casadi::MX pc   = state_vars_[base_x + 2];
    casadi::MX vc   = state_vars_[base_x + 3];

    casadi::DM Xk = casadi::DM::zeros(NX,1);
    for (int i = 0; i < NX; ++i)
        Xk(i) = X_prev[k](i);

    opti_ptr_->set_initial(pcom, Xk(casadi::Slice(0,3)));
    opti_ptr_->set_initial(vcom, Xk(casadi::Slice(3,6)));
    opti_ptr_->set_initial(pc,   Xk(casadi::Slice(6,9)));
    opti_ptr_->set_initial(vc,   Xk(casadi::Slice(9,12)));

    if (k < NH){
    int base_u = 2 * k; 
    // set u initial guess
    auto ac   = input_vars_[base_u + 0];
    auto fc   = input_vars_[base_u + 1];
    
    casadi::DM Uk = casadi::DM::zeros(NU,1);
    for (int i = 0; i < NU; ++i)
      Uk(i) = U_prev[k](i);
    
    opti_ptr_->set_initial(ac,   Uk(casadi::Slice(0,3)));
    opti_ptr_->set_initial(fc,   Uk(casadi::Slice(3,6)));
    }
  }

}