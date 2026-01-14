cmake_minimum_required(VERSION 3.16)
project(mpc_prova)
set(CMAKE_CXX_STANDARD 20)


find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()


# for casadi
# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

# to
add_executable(main
  main.cpp
  MPC.cpp
)

target_include_directories(main
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(main
pinocchio::pinocchio
crocoddyl::crocoddyl
# /usr/local/lib/libcasadi.dylib        # for casadi
)






# ------------------TO SILENT WARNINGS (NOT NECESSARY IF A PROPER INSTALLATION OF DEPENDANCIES HAS BEEN DONE)----------------
# Provide this define to hush hpp-fcl → coal “please migrate” messages in headers
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "" FORCE)
add_compile_definitions(COAL_DISABLE_HPP_FCL_WARNINGS)
# Mute the exact diagnostics that coal/hpp-fcl emit (clang)
target_compile_options(main PRIVATE
  -Wno-deprecated-declarations
  -Wno-macro-redefined
   -Wno-deprecated-enum-enum-conversion
  -Wno-deprecated-anon-enum-enum-conversion)
# -----------------------------------------------------------------------------------------------------------------
