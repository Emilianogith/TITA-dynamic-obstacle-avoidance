// std
// #include <fstream>
// #include <iostream>
// #include <map>

// #include <mujoco/mujoco.h>

#include <WalkingManager.hpp>
#include "MujocoUI.hpp"


void apply_disturbance(mjModel* mj_model_ptr, mjData* mj_data_ptr, int& timestep_counter){
  double point[3]{0.0, 0.0, 0.0};
  double force[3] {110.0, -100.0, 110.0}; // {110.0, -100.0, 110.0}; {-200.0, -160.0, -300.0};
  double torque[3]{0.0, 0.0, 0.0};

  int torso_id = mj_name2id(mj_model_ptr, mjOBJ_BODY, "base_link");

  if (timestep_counter == 2000) {
    mj_applyFT(mj_model_ptr, mj_data_ptr, force, torque, point, torso_id, mj_data_ptr->qfrc_applied);
  }
  if (timestep_counter == 2100) {
    force[0] = -force[0];
    force[1] = -force[1];
    force[2] = -force[2];
    mj_applyFT(mj_model_ptr, mj_data_ptr, force, torque, point, torso_id, mj_data_ptr->qfrc_applied);
  }
}

int main() {
  // Load MJCF (for Mujoco):
  const int kErrorLength = 1024;          // load error string length
  char loadError[kErrorLength] = "";
  const char* mjcf_filepath = "../tita_mj_description/tita.mjcf";
  mjModel* mj_model_ptr = mj_loadXML(mjcf_filepath, nullptr, loadError, kErrorLength);
  if (!mj_model_ptr) {
    std::cerr << "Error loading model: " << loadError << std::endl;
    return -1;
  }
  mjData* mj_data_ptr = mj_makeData(mj_model_ptr);


  // log files
  std::ofstream joint_vel_log_file("/tmp/joint_vel.txt");
  std::ofstream joint_eff_log_file("/tmp/joint_eff.txt");


  // Init robot posture:
  mjtNum joint_left_leg_1_init = 0.0;
  mjtNum joint_left_leg_2_init = 0.5;   // 0.25; (up-position)
  mjtNum joint_left_leg_3_init = -1.0;  // -0.5; (up-position)
  mjtNum joint_left_leg_4_init = 0.0;
  mjtNum joint_right_leg_1_init = 0.0;
  mjtNum joint_right_leg_2_init = 0.5;
  mjtNum joint_right_leg_3_init = -1.0;
  mjtNum joint_right_leg_4_init = 0.0;

  mj_data_ptr->qpos[0] = 0.0;                                     // x
  mj_data_ptr->qpos[1] = 0.0;                                     // y
  mj_data_ptr->qpos[2] = 0.399 + 0.05 - 0.005; // +0.02;(up-position) //-0.3;(upside-down-position) // z
  mj_data_ptr->qpos[3] = 1.0;                                     // η
  mj_data_ptr->qpos[4] = 0.0; //1.0 for upside down               // ε_x
  mj_data_ptr->qpos[5] = 0.0;                                     // ε_y
  mj_data_ptr->qpos[6] = 0.0;                                     // ε_z
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_left_leg_1")]] = joint_left_leg_1_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_left_leg_2")]] = joint_left_leg_2_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_left_leg_3")]] = joint_left_leg_3_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_left_leg_4")]] = joint_left_leg_4_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_right_leg_1")]] = joint_right_leg_1_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_right_leg_2")]] = joint_right_leg_2_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_right_leg_3")]] = joint_right_leg_3_init;
  mj_data_ptr->qpos[mj_model_ptr->jnt_qposadr[mj_name2id(mj_model_ptr, mjOBJ_JOINT, "joint_right_leg_4")]] = joint_right_leg_4_init;

  mjtNum* qpos0 = (mjtNum*) malloc(sizeof(mjtNum) * mj_model_ptr->nq);
  memcpy(qpos0, mj_data_ptr->qpos, mj_model_ptr->nq * sizeof(mjtNum));
  

  // extracting armatures values from the simulation
  std::map<std::string, double> armatures;
  for (int i = 0; i < mj_model_ptr->nu; ++i) {
    int joint_id = mj_model_ptr->actuator_trnid[i * 2];
    std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
    int dof_id = mj_model_ptr->jnt_dofadr[joint_id];
    armatures[joint_name] = mj_model_ptr->dof_armature[dof_id];
  }


  // Walking Manager:
  labrob::RobotState initial_robot_state = labrob::robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
  labrob::WalkingManager walking_manager;
  walking_manager.init(initial_robot_state, armatures);


  // // zero gravity
  // mj_model_ptr->opt.gravity[0] = 0.0;
  // mj_model_ptr->opt.gravity[1] = 0.0;
  // mj_model_ptr->opt.gravity[2] = 0.0;

  
  // Mujoco UI
  auto& mujoco_ui = *labrob::MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);

  double dt = mj_model_ptr->opt.timestep;   // simulation timestep
  std::cout<< "simulation dt: " << dt << std::endl;
  std::cout<< "simulation freq: " << 1.0/dt << std::endl;

  static int framerate = 60.0;
  bool first_frame = false;

  int timestep_counter = 0;

  // Simulation loop:
  while (!mujoco_ui.windowShouldClose()) {

  auto start_time = std::chrono::high_resolution_clock::now();

  mjtNum simstart = mj_data_ptr->time;
  while( mj_data_ptr->time - simstart < 1.0/framerate ) { // non serve

    labrob::RobotState robot_state = labrob::robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
    
    // Walking manager
    labrob::JointCommand joint_command;
    walking_manager.update(robot_state, joint_command);

    // apply a disturbance
    // apply_disturbance(mj_model_ptr, mj_data_ptr, timestep_counter);
    ++timestep_counter;
    
    mj_step1(mj_model_ptr, mj_data_ptr);
    
    if (first_frame == true) {
      mujoco_ui.render();
      continue;
    }

    for (int i = 0; i < mj_model_ptr->nu; ++i) {
      int joint_id = mj_model_ptr->actuator_trnid[i * 2];
      std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
      int jnt_qvel_idx = mj_model_ptr->jnt_dofadr[joint_id];
      mj_data_ptr->ctrl[i] = joint_command[joint_name];

      joint_vel_log_file << mj_data_ptr->qvel[jnt_qvel_idx] << " ";
      joint_eff_log_file << mj_data_ptr->ctrl[i] << " ";
    }


    //  for (int i = 0; i < mj_data_ptr->ncon; i++)
    //     {
    //         const mjContact& con = mj_data_ptr->contact[i];

    //         std::cout << "Contact " << i << ": ("
    //                   << con.pos[0] << ", "
    //                   << con.pos[1] << ", "
    //                   << con.pos[2] << ")\n";
    //     }


    mj_step2(mj_model_ptr, mj_data_ptr);

    
    joint_vel_log_file << std::endl;
    joint_eff_log_file << std::endl;
    
    }

  double end_sim = mj_data_ptr->time;
  // Fine misurazione del tempo
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // Stampa del tempo di esecuzione
  std::cout << "Controller period: " << duration << " microseconds" << std::endl;
  
  
  double sim_elapsed = end_sim - simstart;
  double real_elapsed = std::chrono::duration<double>(end_time - start_time).count();
  double RTF = sim_elapsed / real_elapsed;
  std::cout << "Simulated time: " << sim_elapsed << std::endl;
  std::cout << "Real time: " << real_elapsed << std::endl;
  std::cout << "Real-time factor: " << RTF << std::endl;

  mujoco_ui.render();
  }

  // Free memory (Mujoco):
  mj_deleteData(mj_data_ptr);
  mj_deleteModel(mj_model_ptr);

  joint_vel_log_file.close();
  joint_eff_log_file.close();

  return 0;
}