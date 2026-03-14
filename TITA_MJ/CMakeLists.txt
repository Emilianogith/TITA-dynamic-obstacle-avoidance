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
set(blasfeo_INCLUDE_DIRS /Users/emilianoparadiso/.local/blasfeo/include)
set(hpipm_INCLUDE_DIRS /Users/emilianoparadiso/.local/hpipm/include)
set(blasfeo_LIBRARIES /Users/emilianoparadiso/.local/blasfeo/lib/libblasfeo.a)
set(hpipm_LIBRARIES /Users/emilianoparadiso/.local/hpipm/lib/libhpipm.a)

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