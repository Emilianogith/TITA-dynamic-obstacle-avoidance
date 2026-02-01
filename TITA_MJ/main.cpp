// std
// #include <fstream>
// #include <iostream>
// #include <map>

// #include <mujoco/mujoco.h>

#include <WalkingManager.hpp>
#include "MujocoUI.hpp"

#include <cstring>


static inline void mat3_mul_vec3(const mjtNum R[9], const mjtNum v[3], mjtNum out[3]) {
  out[0] = R[0]*v[0] + R[3]*v[1] + R[6]*v[2];
  out[1] = R[1]*v[0] + R[4]*v[1] + R[7]*v[2];
  out[2] = R[2]*v[0] + R[5]*v[1] + R[8]*v[2];
}

void print_contacts(const mjModel* m, const mjData* d) {


  Eigen::Vector3d f_r = Eigen::Vector3d::Zero();
  Eigen::Vector3d f_l = Eigen::Vector3d::Zero();
  
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& con = d->contact[i];

    // contact force/torque in contact frame
    mjtNum cf[6];
    mj_contactForce(m, d, i, cf);

    // names (optional)
    const char* g1 = mj_id2name(m, mjOBJ_GEOM, con.geom1);
    const char* g2 = mj_id2name(m, mjOBJ_GEOM, con.geom2);
    if (!g1) g1 = "(null)";
    if (!g2) g2 = "(null)";

    // convert force and torque to world frame (optional)
    mjtNum f_local[3] = {cf[0], cf[1], cf[2]};
    mjtNum t_local[3] = {cf[3], cf[4], cf[5]};
    mjtNum f_world[3], t_world[3];
    mat3_mul_vec3(con.frame, f_local, f_world);
    mat3_mul_vec3(con.frame, t_local, t_world);

    std::printf("friction:\n");
    std::printf("[%.6f %.6f %.6f %.6f %.6f]\n", (double)con.friction[0], (double)con.friction[1], (double)con.friction[2], (double)con.friction[3], (double)con.friction[4]);

    std::printf("con.dim:\n");
    std::printf("[%.6f]\n", (double)con.dim);

  auto dump_geom = [&](int gid){
  const mjtNum* fr = m->geom_friction + 3*gid;
  printf("geom %d (%s): fric=[%.3f %.3f %.3f], condim=%d\n",
         gid, mj_id2name(m, mjOBJ_GEOM, gid),
         (double)fr[0], (double)fr[1], (double)fr[2],
         m->geom_condim[gid]);
  };

  dump_geom(con.geom1);
  dump_geom(con.geom2);

    std::printf(
      "contact %d: %s <-> %s | pos=[%.3f %.3f %.3f] | "
      "F_local=[%.3f %.3f %.3f]  T_local=[%.3f %.3f %.3f] | "
      "F_world=[%.3f %.3f %.3f]\n",
      i, g1, g2,
      con.pos[0], con.pos[1], con.pos[2],
      (double)cf[0], (double)cf[1], (double)cf[2],
      (double)cf[3], (double)cf[4], (double)cf[5],
      (double)f_world[0], (double)f_world[1], (double)f_world[2]
    );

    if (g2 && std::strcmp(g2, "left_leg_4_collision") == 0) {
      f_l += Eigen::Vector3d(f_world[0], f_world[1], f_world[2]);
    }
    if (g2 && std::strcmp(g2, "right_leg_4_collision") == 0) {
      f_r += Eigen::Vector3d(f_world[0], f_world[1], f_world[2]);
    }

  }

  std::cout << "f_l from mujoco " << f_l << std::endl;
  std::cout << "f_r from mujoco " << f_r << std::endl;
}



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


labrob::RobotState robot_state_from_mujoco(mjModel* m, mjData* d) {
labrob::RobotState robot_state;

robot_state.position = Eigen::Vector3d(
  d->qpos[0], d->qpos[1], d->qpos[2]
);

robot_state.orientation = Eigen::Quaterniond(
    d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]
);

robot_state.linear_velocity = robot_state.orientation.toRotationMatrix().transpose() *
    Eigen::Vector3d(
        d->qvel[0], d->qvel[1], d->qvel[2]
    );

robot_state.angular_velocity = Eigen::Vector3d(
  d->qvel[3], d->qvel[4], d->qvel[5]
);

for (int i = 1; i < m->njnt; ++i) {
  const char* name = mj_id2name(m, mjOBJ_JOINT, i);
  robot_state.joint_state[name].pos = d->qpos[m->jnt_qposadr[i]];
  robot_state.joint_state[name].vel = d->qvel[m->jnt_dofadr[i]];
}

static double force[6];
static double result[3];
Eigen::Vector3d sum = Eigen::Vector3d::Zero();
robot_state.contact_points.resize(d->ncon);
robot_state.contact_forces.resize(d->ncon);
for (int i = 0; i < d->ncon; ++i) {
  mj_contactForce(m, d, i, force);
  //mju_rotVecMatT(result, force, d->contact[i].frame);
  mju_mulMatVec(result, d->contact[i].frame, force, 3, 3);
  for (int row = 0; row < 3; ++row) {
      result[row] = 0;
      for (int col = 0; col < 3; ++col) {
          result[row] += d->contact[i].frame[3 * col + row] * force[col];
      }
  }
  sum += Eigen::Vector3d(result);
  for (int j = 0; j < 3; ++j) {
    robot_state.contact_points[i](j) = d->contact[i].pos[j];
    robot_state.contact_forces[i](j) = result[j];
  }
}

robot_state.total_force = sum;

return robot_state;
}

int main() {
  // Load MJCF (for Mujoco):
  const int kErrorLength = 1024;          // load error string length
  char loadError[kErrorLength] = "";
  const char* mjcf_filepath = "../tita_mj_description/tita_world.xml";
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
  mj_data_ptr->qpos[2] = 0.399 + 0.05 - 0.005 - 0.001; // +0.02;(up-position) //-0.3;(upside-down-position) // z
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
  labrob::RobotState initial_robot_state = robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
  labrob::WalkingManager walking_manager;
  walking_manager.init(initial_robot_state, armatures);


  // // zero gravity
  // mj_model_ptr->opt.gravity[0] = 0.0;
  // mj_model_ptr->opt.gravity[1] = 0.0;
  // mj_model_ptr->opt.gravity[2] = 0.0;

  
  // Mujoco UI
  auto& mujoco_ui = *labrob::MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);

  double dt = mj_model_ptr->opt.timestep;   // simulation timestep

  static int framerate = 60.0;
  bool first_frame = false;

  int timestep_counter = 0;

  // Simulation loop:
  while (!mujoco_ui.windowShouldClose()) {

  auto start_time = std::chrono::high_resolution_clock::now();

  mjtNum simstart = mj_data_ptr->time;
  while( mj_data_ptr->time - simstart < 1.0/framerate ) { // non serve
    
    mj_step1(mj_model_ptr, mj_data_ptr);
    labrob::RobotState robot_state = robot_state_from_mujoco(mj_model_ptr, mj_data_ptr);
    
    // Walking manager
    labrob::JointCommand joint_torque;
    labrob::JointCommand joint_acceleration;
    walking_manager.update(robot_state, joint_torque, joint_acceleration);

    // apply a disturbance
    // apply_disturbance(mj_model_ptr, mj_data_ptr, timestep_counter);
    ++timestep_counter;

    if (first_frame == true) {
      mujoco_ui.render();
      continue;
    }

    for (int i = 0; i < mj_model_ptr->nu; ++i) {
      int joint_id = mj_model_ptr->actuator_trnid[i * 2];
      std::string joint_name = std::string(mj_id2name(mj_model_ptr, mjOBJ_JOINT, joint_id));
      int jnt_qvel_idx = mj_model_ptr->jnt_dofadr[joint_id];
      mj_data_ptr->ctrl[i] = joint_torque[joint_name];

      joint_vel_log_file << mj_data_ptr->qvel[jnt_qvel_idx] << " ";
      joint_eff_log_file << mj_data_ptr->ctrl[i] << " ";
    }

    mj_step2(mj_model_ptr, mj_data_ptr);

  
    // print_contacts(mj_model_ptr, mj_data_ptr);

    // Eigen::Map<const Eigen::VectorXd> qacc(mj_data_ptr->qacc, mj_model_ptr->nv);
    // std::cout << "qacc = " << qacc.transpose() << std::endl;
    
    
    joint_vel_log_file << std::endl;
    joint_eff_log_file << std::endl;
    
    }

  double end_sim = mj_data_ptr->time;
  // Fine misurazione del tempo
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // Stampa del tempo di esecuzione
  // std::cout << "Controller period: " << duration << " us" << std::endl;
  
  
  // double sim_elapsed = end_sim - simstart;
  // double real_elapsed = std::chrono::duration<double>(end_time - start_time).count();
  // double RTF = sim_elapsed / real_elapsed;
  // std::cout << "Simulated time: " << sim_elapsed << std::endl;
  // std::cout << "Real time: " << real_elapsed << std::endl;
  // std::cout << "Real-time factor: " << RTF << std::endl;

  mujoco_ui.render();
  }

  // Free memory (Mujoco):
  mj_deleteData(mj_data_ptr);
  mj_deleteModel(mj_model_ptr);

  joint_vel_log_file.close();
  joint_eff_log_file.close();

  return 0;
}