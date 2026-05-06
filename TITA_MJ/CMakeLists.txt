cmake_minimum_required(VERSION 3.16)
project(TITA_MJ)

set(CMAKE_CXX_STANDARD 17)


find_package(glfw3 REQUIRED)
find_package(mujoco REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

# Manually set INCLUDE_DIRS and LIBRARIES since not handled in blasfeo and hpipm
set(blasfeo_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/blasfeo/Include)
set(hpipm_INCLUDE_DIRS  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/hpipm/Include)
set(blasfeo_LIBRARIES   ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/blasfeo/Lib/libblasfeo.a)
set(hpipm_LIBRARIES     ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/hpipm/Lib/libhpipm.a)

# IPOPT headers are installed under include/coin-or/ — add that directory so
# that crocoddyl's ipopt.hpp can find <IpIpoptApplication.hpp> without prefix.
find_path(IPOPT_INCLUDE_DIR IpIpoptApplication.hpp
  PATH_SUFFIXES coin-or coin
)
if(NOT IPOPT_INCLUDE_DIR)
  message(FATAL_ERROR "IPOPT headers not found. Install ipopt in your pixi environment.")
endif()

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
    ${IPOPT_INCLUDE_DIR}
)

target_link_libraries(UtilsLib
  pinocchio::pinocchio
  ${hpipm_LIBRARIES}
  ${blasfeo_LIBRARIES}
  crocoddyl::crocoddyl
)

add_executable(main
  main.cpp
)
target_include_directories(main
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(main
  glfw
  mujoco::mujoco
  UtilsLib
)


# add_executable(main_with_Filter
#   main_with_Filter.cpp
# )
# target_include_directories(main_with_Filter
#   PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# target_link_libraries(main_with_Filter
#   glfw
#   mujoco::mujoco
#   UtilsLib
# )
# target_compile_options(main_with_Filter PRIVATE
#   -Wno-deprecated-declarations
#   -Wno-macro-redefined)






# ------------------TO SILENT WARNINGS (NOT NECESSARY IF A PROPER INSTALLATION OF DEPENDANCIES HAS BEEN DONE)----------------
# Provide this define to hush hpp-fcl → coal “please migrate” messages in headers
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "" FORCE)
add_compile_definitions(COAL_DISABLE_HPP_FCL_WARNINGS)
# Mute the exact diagnostics that coal/hpp-fcl emit (clang)
target_compile_options(UtilsLib PRIVATE
  -Wno-deprecated-declarations
  -Wno-macro-redefined)
target_compile_options(main PRIVATE
  -Wno-deprecated-declarations
  -Wno-macro-redefined)
# -----------------------------------------------------------------------------------------------------------------