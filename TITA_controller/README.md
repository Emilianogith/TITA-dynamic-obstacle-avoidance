This is a package for the TITA robot control



## How to run the code
1. create a ros2 workspace 
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

2. clone this package inside the src 
git clone ..TITA_controller

3. compile
colcon build

4. source the bash
source install/setup.bash

5. ros2 run TITA_controller controller_node