# Manual acceptance drivers: production bridge and unchanged RL objects.
# Build outside the workspace. RL_ROOT must have an existing go2_ctrl build;
# only DDS initialization is wrapped to give native and ROS identical isolation.
cmake_minimum_required(VERSION 3.16)
project(go2_sim_acceptance)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
set(SIM "$ENV{UNITREE_MUJOCO_ROOT}")
set(MJ "$ENV{MUJOCO_ROOT}")
set(RL_ROOT "" CACHE PATH "RL repository with compiled deploy/robots/go2")
set(SDK_ROOT "" CACHE PATH "Unitree SDK2 repository")
set(RL "${RL_ROOT}/deploy/robots/go2")
set(ORT "${RL_ROOT}/deploy/thirdparty/onnxruntime-linux-aarch64-gpu-1.16.0")
add_executable(acceptance_simulator simulator.cpp "${SIM}/simulate/src/joystick/joystick.cc")
target_include_directories(acceptance_simulator PRIVATE
  "${SIM}/simulate/src" "${MJ}/include" "/usr/local/include/iceoryx/v2.0.2")
target_link_directories(acceptance_simulator PRIVATE "${MJ}/lib")
target_link_libraries(acceptance_simulator PRIVATE
  mujoco unitree_sdk2 yaml-cpp Boost::program_options fmt pthread)
set_target_properties(acceptance_simulator PROPERTIES
  BUILD_RPATH "${MJ}/lib;/opt/unitree_robotics/lib")
enable_testing()
add_executable(acceptance_clock_test clock_test.cpp)
target_include_directories(acceptance_clock_test PRIVATE "${SIM}/simulate/src" "${MJ}/include")
target_link_directories(acceptance_clock_test PRIVATE "${MJ}/lib")
target_link_libraries(acceptance_clock_test PRIVATE mujoco)
set_target_properties(acceptance_clock_test PROPERTIES BUILD_RPATH "${MJ}/lib")
add_test(NAME acceptance_clock_test COMMAND acceptance_clock_test
  "${CMAKE_CURRENT_SOURCE_DIR}/clock_fixture.xml")
add_executable(acceptance_controller
  rl_discovery.cpp "${RL}/build/CMakeFiles/go2_ctrl.dir/main.cpp.o")
target_include_directories(acceptance_controller PRIVATE "${SDK_ROOT}/include"
  "${SDK_ROOT}/thirdparty/include" "${SDK_ROOT}/thirdparty/include/ddscxx")
target_link_directories(acceptance_controller PRIVATE
  "${SDK_ROOT}/lib/aarch64" "${SDK_ROOT}/thirdparty/lib/aarch64")
target_link_libraries(acceptance_controller PRIVATE "${RL}/build/libgo2_controller_lib.a"
  unitree_sdk2 ddsc ddscxx rt pthread libboost_program_options.a libyaml-cpp.a fmt
  "${ORT}/lib/libonnxruntime.so.1.16.0")
target_link_options(acceptance_controller PRIVATE
  "-Wl,--wrap=_ZN7unitree5robot14ChannelFactory4InitEiRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE")
set_target_properties(acceptance_controller PROPERTIES OUTPUT_NAME go2_ctrl
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/rlroot/deploy/robots/go2/build"
  BUILD_RPATH "${SDK_ROOT}/lib/aarch64;${SDK_ROOT}/thirdparty/lib/aarch64;${ORT}/lib")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/rlroot")
file(CREATE_LINK "${RL_ROOT}/logs" "${CMAKE_BINARY_DIR}/rlroot/logs" SYMBOLIC)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/rlroot/deploy/robots/go2")
file(CREATE_LINK "${RL}/config"
  "${CMAKE_BINARY_DIR}/rlroot/deploy/robots/go2/config" SYMBOLIC)
