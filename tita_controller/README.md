# TITA Controller

This package provides the control software for the **TITA robot** based on **ROS 2 Control**.
To load the low-level motor controller, please refer to the following repository: https://github.com/DDTRobot/tita_hardware_ros2_control.

Then launch the hardware bring-up with:

```bash
ros2 launch hw_bringup hw_bringup.launch.py
```
# How to run the code

## Setup
1. Create a ROS2 workspace
```bash
mkdir -p ~/Desktop/ros2_ws/src
cd ~/Desktop/ros2_ws/src
```
2. clone this repository
```bash
git clone https://github.com/Emilianogith/TITA-dynamic-obstacle-avoidance/tita_controller.git
```
3. Build the workspace
```bash
cd ~/Desktop/ros2_ws
colcon build
source install/setup.bash
```



## Usage

### Run the controller
The controller is handled through a controller manager.  
To start it, run the following commands in two separate terminals:
```bash
ros2 run tita_controller controller_node
ros2 run tita_controller controller_manager
```
Once the controller manager is running, use the following commands:
- Press 0 → Security Stop
- Press 1 → Regulation Mode
- Press 2 → Start Filter
- Press 3 → Whole Body Mode (after starting filter)


### Run the robot simulator
A node for simulating the robot sensors is available for testing.
Store the log data in: 
```bash
~/Desktop/ros2_ws/src/tita_controller/robot_data
```
then run 
```bash
ros2 run tita_controller robot_simulator
```
