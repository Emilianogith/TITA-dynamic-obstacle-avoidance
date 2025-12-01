cmake_minimum_required(VERSION 3.16)
project(mpc_prova)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -funroll-loops -ffast-math -flto")
set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

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
