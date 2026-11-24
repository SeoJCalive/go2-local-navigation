# Manual W02/W03 integration drivers using the pinned Go2Bridge without a GUI.
cmake_minimum_required(VERSION 3.16)
project(w02_headless)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
set(SIM "$ENV{UNITREE_MUJOCO_ROOT}")
set(MJ "$ENV{MUJOCO_ROOT}")
add_executable(w02_headless headless_simulator.cpp "${SIM}/simulate/src/joystick/joystick.cc")
add_executable(w03_headless bringup_simulator.cpp "${SIM}/simulate/src/joystick/joystick.cc")
foreach(target w02_headless w03_headless)
target_include_directories(${target} PRIVATE
  "${SIM}/simulate/src" "${MJ}/include" "/usr/local/include/iceoryx/v2.0.2")
target_link_directories(${target} PRIVATE "${MJ}/lib")
target_link_libraries(${target} PRIVATE
  mujoco unitree_sdk2 yaml-cpp Boost::program_options fmt pthread)
set_target_properties(${target} PROPERTIES BUILD_RPATH "${MJ}/lib;/opt/unitree_robotics/lib")
endforeach()
