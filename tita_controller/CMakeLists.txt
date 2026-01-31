cmake_minimum_required(VERSION 3.8)
project(tita_controller)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build." FORCE)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_definitions(EIGEN_NO_DEBUG)
  add_compile_options(-O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_package(Eigen3 REQUIRED)
# find_package(pinocchio REQUIRED)

find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)


# Manually set INCLUDE_DIRS and LIBRARIES since not handled in blasfeo and hpipm
set(blasfeo_INCLUDE_DIRS /opt/blasfeo/include)
set(hpipm_INCLUDE_DIRS /opt/hpipm/include)
set(blasfeo_LIBRARIES /opt/blasfeo/lib/libblasfeo.a)
set(hpipm_LIBRARIES /opt/hpipm/lib/libhpipm.a)


add_library(
  UtilsLib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/JointState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/RobotState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/JointCommand.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SE3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/WholeBodyController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/WalkingManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MPC.cpp
)
target_include_directories(UtilsLib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${hpipm_INCLUDE_DIRS}
    ${blasfeo_INCLUDE_DIRS}
)

target_link_libraries(UtilsLib
  pinocchio::pinocchio
  ${hpipm_LIBRARIES}
  ${blasfeo_LIBRARIES}
  crocoddyl::crocoddyl
)


add_executable(topic_reader src/topic_reader.cpp)
ament_target_dependencies(topic_reader rclcpp std_msgs nav_msgs sensor_msgs)

add_executable(robot_simulator src/robot_simulator.cpp)
ament_target_dependencies(robot_simulator rclcpp std_msgs nav_msgs sensor_msgs)


# controller node
add_executable(controller_node 
  src/controller_node.cpp
)

target_link_libraries(controller_node 
  glfw
  UtilsLib
)

target_include_directories(controller_node 
 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(controller_node 
rclcpp 
nav_msgs
sensor_msgs
tf2
tf2_ros
geometry_msgs
)


install(TARGETS
  topic_reader
  controller_node
  robot_simulator
  DESTINATION lib/${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  python_src/save_urdf.py           # for savingbthe urdf
  python_src/send_urdf.py           # for testng 
  DESTINATION lib/${PROJECT_NAME}   # this is where ROS 2 expects python executables
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
