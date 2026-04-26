# TITA_MJ — MuJoCo Simulation Environment

This repository contains the **MuJoCo-based simulation framework** for the TITA two-wheel legged robot, developed for testing and validating **acrobatic obstacle avoidance through jumping maneuvers**.

The simulation is used to evaluate the full control stack, including trajectory generation, MPC-based control, whole-body control, and real-time obstacle avoidance.

---


## Build Instructions

From the root of the `TITA_MJ` directory:

```bash id="build_mj_01"
mkdir -p build
cd build
cmake ..
make
```

After successful compilation, run the simulation using:
```
./main
```
