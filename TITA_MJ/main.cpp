// std
// #include <fstream>
// #include <iostream>
// #include <map>

// #include <mujoco/mujoco.h>

#include <WholeBodyController.hpp>
#include "MujocoUI.hpp"



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

  // print the joint names
  // std::cout << "Total number of generalized variables:" << mj_model_ptr->nv << std::endl;
//   for (int j = 0; j < mj_model_ptr->njnt; ++j) {
//     std::cout << mj_model_ptr->names + mj_model_ptr->name_jntadr[j] 
//               << " → qpos[" << mj_model_ptr->jnt_qposadr[j] << "]" << std::endl;
// }


  // log files
  std::ofstream joint_vel_log_file("/tmp/joint_vel.txt");
  std::ofstream joint_eff_log_file("/tmp/joint_eff.txt");


  // Init robot posture:
  mjtNum joint_left_leg_1_init = 0.0;
  mjtNum joint_left_leg_2_init = -0.5;
  mjtNum joint_left_leg_3_init = 1.0;
  mjtNum joint_left_leg_4_init = 0.0;
  mjtNum joint_right_leg_1_init = 0.0;
  mjtNum joint_right_leg_2_init = -0.5;
  mjtNum joint_right_leg_3_init = 1.0;
  mjtNum joint_right_leg_4_init = 0.0;

  mj_data_ptr->qpos[0] = 0.0;                                     // x
  mj_data_ptr->qpos[1] = 0.0;                                     // y
  mj_data_ptr->qpos[2] = 0.399-0.05 -0.005;                                   // z
  mj_data_ptr->qpos[3] = 1.0;                                     // η
  mj_data_ptr->qpos[4] = 0.0;                                     // ε_x
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
  
  
  // Debug
  // for (int i = 0; i <mj_model_ptr->nq ; ++i) {
  //     std::cout << "Value for the joint" << i << " " << mj_data_ptr->qpos[i] << std::endl;
  //   }
  mj_step1(mj_model_ptr, mj_data_ptr);
  
  std::map<std::string, double> armatures;
  for (int i = 0; i < mj_model_ptr->nu; ++i) {
    int joint_id = mj_model_ptr->actuator_trnid[i * 2];
    std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
    int dof_id = mj_model_ptr->jnt_dofadr[joint_id];
    armatures[joint_name] = mj_model_ptr->dof_armature[dof_id];
  }


  // Controller:
  labrob::RobotState initial_robot_state = labrob::robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
  std::shared_ptr<labrob::WholeBodyController> whole_body_controller_ptr = std::make_shared<labrob::WholeBodyController>(
      initial_robot_state, armatures
  );


  // Desired configuration:
  labrob::DesiredConfiguration des_configuration;
  des_configuration.qjnt = Eigen::VectorXd::Zero(mj_model_ptr->nu);
  des_configuration.qjnt << 
    0.0,   // joint_left_leg_1
   -0.5,   // joint_left_leg_2
    1.5,   // joint_left_leg_3
    0.0,   // joint_left_leg_4
    0.0,   // joint_right_leg_1
   -0.5,   // joint_right_leg_2
    1.5,   // joint_right_leg_3
    0.0;   // joint_right_leg_4
  des_configuration.qjntdot = Eigen::VectorXd::Zero(mj_model_ptr->nu);
  des_configuration.qjntddot = Eigen::VectorXd::Zero(mj_model_ptr->nu);
  // des_configuration.position = Eigen::Vector3d(0.0, 0.0, 0.2);
  // des_configuration.orientation = Eigen::Quaterniond::Identity();
  // des_configuration.linear_velocity = Eigen::Vector3d::Zero();
  // des_configuration.angular_velocity = Eigen::Vector3d::Zero();
  des_configuration.com.pos = Eigen::Vector3d(0.0, 0.0, 20.0);  
  des_configuration.com.vel = Eigen::Vector3d::Zero();
  des_configuration.com.acc = Eigen::Vector3d::Zero();
  des_configuration.lwheel.pos.p = Eigen::Vector3d(0.01, 0.27, 0.0);
  des_configuration.lwheel.pos.R = Eigen::Matrix3d::Identity();
  des_configuration.lwheel.vel = Eigen::Vector<double, 6>::Zero();
  des_configuration.lwheel.acc = Eigen::Vector<double, 6>::Zero();
  des_configuration.rwheel.pos.p = Eigen::Vector3d(0.01, -0.27, 0.0);
  des_configuration.rwheel.pos.R = Eigen::Matrix3d::Identity();
  des_configuration.rwheel.vel = Eigen::Vector<double, 6>::Zero();
  des_configuration.rwheel.acc = Eigen::Vector<double, 6>::Zero();
  des_configuration.base_link.pos =Eigen::Matrix3d::Identity();
  des_configuration.base_link.vel = Eigen::Vector3d::Zero();
  des_configuration.base_link.acc = Eigen::Vector3d::Zero();



  
  // Mujoco UI
  auto& mujoco_ui = *labrob::MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);

  static int framerate = 60.0;


  // slow down:
  bool slow_down = true;
  bool first_frame = false;
  int count = 0;

  // Simulation loop:
  while (!mujoco_ui.windowShouldClose()) {

    auto start_time = std::chrono::high_resolution_clock::now();

    mjtNum simstart = mj_data_ptr->time;
    while( mj_data_ptr->time - simstart < 1.0/framerate ) {
      labrob::RobotState robot_state = labrob::robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
      
      // WBC
      labrob::JointCommand joint_command;
      joint_command = whole_body_controller_ptr->compute_inverse_dynamics(robot_state, des_configuration);
      // walking_manager.update(robot_state, joint_command);
      
      
      if (first_frame == true) {break;}

      mj_step1(mj_model_ptr, mj_data_ptr);

      for (int i = 0; i < mj_model_ptr->nu; ++i) {
        int joint_id = mj_model_ptr->actuator_trnid[i * 2];
        std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
        int jnt_qvel_idx = mj_model_ptr->jnt_dofadr[joint_id];
        mj_data_ptr->ctrl[i] = joint_command[joint_name];

        joint_vel_log_file << mj_data_ptr->qvel[jnt_qvel_idx] << " ";
        joint_eff_log_file << mj_data_ptr->ctrl[i] << " ";
      }

      mj_step2(mj_model_ptr, mj_data_ptr);

      // slow down the simulation:
      if (slow_down == true) {
        if (count<10){
          break;
        };
        ++count;
      }

      joint_vel_log_file << std::endl;
      joint_eff_log_file << std::endl;
    
    }

    // Fine misurazione del tempo
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Stampa del tempo di esecuzione
    // std::cout << "Tempo di esecuzione del main: " << duration << " millisecondi" << std::endl;

    mujoco_ui.render();
  }

  // Free memory (Mujoco):
  mj_deleteData(mj_data_ptr);
  mj_deleteModel(mj_model_ptr);

  joint_vel_log_file.close();
  joint_eff_log_file.close();

  return 0;
}

