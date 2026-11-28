# Build MuJoCo sensor runtime and independent state/planar checks; install ROS assets.
# Flow: resolve pinned MuJoCo/Unitree inputs, build runtime, register ament tests.
# Outputs: package executables, assets and CTest/pytest results.
# External contracts: third_party/unitree_mujoco/README.md and test/wall_hug/README.md.
# G02 diagnostics stay in the repo; no Karto patch or dataset is installed.
cmake_minimum_required(VERSION 3.16)
project(go2_mujoco_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(go2_sim_map_display SHARED src/transient_map_display.cpp)
ament_target_dependencies(go2_sim_map_display rviz_default_plugins pluginlib)
target_link_libraries(go2_sim_map_display Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)
install(TARGETS go2_sim_map_display LIBRARY DESTINATION lib)

set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}" CACHE PATH "MuJoCo distribution root")
set(UNITREE_MUJOCO_ROOT "$ENV{UNITREE_MUJOCO_ROOT}" CACHE PATH "Unitree MuJoCo repository root")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY mujoco HINTS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
find_path(UNITREE_MUJOCO_ROS2_INCLUDE_DIR motor_crc.h
  HINTS "${UNITREE_MUJOCO_ROOT}/example/ros2/include" REQUIRED)
find_file(UNITREE_MUJOCO_MOTOR_CRC_SOURCE motor_crc.cpp
  HINTS "${UNITREE_MUJOCO_ROOT}/example/ros2/src" REQUIRED)
find_file(GO2_XML go2.xml
  HINTS "${UNITREE_MUJOCO_ROOT}/unitree_robots/go2" REQUIRED)
set(GO2_ASSETS "${UNITREE_MUJOCO_ROOT}/unitree_robots/go2/assets")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/scene")
configure_file(
  scene/go2_indoor.xml.in
  "${CMAKE_CURRENT_BINARY_DIR}/scene/go2_indoor.xml"
  @ONLY
)

add_executable(mujoco_lidar_node
  src/lidar_model.cpp
  src/mujoco_lidar_node.cpp
)
target_include_directories(mujoco_lidar_node PRIVATE include "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(mujoco_lidar_node "${MUJOCO_LIBRARY}")
ament_target_dependencies(mujoco_lidar_node rclcpp nav_msgs rosgraph_msgs sensor_msgs unitree_go)
set_target_properties(mujoco_lidar_node PROPERTIES
  BUILD_RPATH "${MUJOCO_ROOT}/lib"
  INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)

add_executable(go2_mujoco_upright_hold
  src/upright_hold_node.cpp
  "${UNITREE_MUJOCO_MOTOR_CRC_SOURCE}"
)
target_include_directories(go2_mujoco_upright_hold PRIVATE
  "${UNITREE_MUJOCO_ROS2_INCLUDE_DIR}")
ament_target_dependencies(go2_mujoco_upright_hold rclcpp unitree_go)

install(TARGETS mujoco_lidar_node go2_mujoco_upright_hold
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_save_map DESTINATION lib/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/scene/go2_indoor.xml"
  DESTINATION share/${PROJECT_NAME}/scene)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(CycloneDDS REQUIRED)
  add_executable(acceptance_posture_test test/acceptance/posture_test.cpp)
  add_test(NAME acceptance_posture_test COMMAND acceptance_posture_test)
  ament_add_gtest(wall_hug_clearance_test test/test_wall_hug_clearance.cpp)
  target_include_directories(wall_hug_clearance_test PRIVATE test "${MUJOCO_INCLUDE_DIR}")
  target_link_libraries(wall_hug_clearance_test "${MUJOCO_LIBRARY}")
  ament_add_pytest_test(wall_hug_policy test/test_wall_hug_policy.py)
  ament_add_pytest_test(wall_hug_async test/test_wall_hug_async.py TIMEOUT 90)
  ament_add_pytest_test(wall_hug_video test/test_wall_hug_video.py)
  ament_add_pytest_test(wall_hug_geometry test/test_wall_hug_geometry.py)
  ament_add_pytest_test(wall_hug_pose test/test_wall_hug_pose.py)
  ament_add_pytest_test(wall_hug_replay test/test_wall_hug_replay.py TIMEOUT 180
    ENV "OPENBLAS_NUM_THREADS=1" "OMP_NUM_THREADS=1")
  ament_add_pytest_test(wall_hug_counterfactual test/test_wall_hug_counterfactual.py TIMEOUT 900
    ENV "OPENBLAS_NUM_THREADS=1" "OMP_NUM_THREADS=1")
  # 목적: built runtime SDK의 재생 동등성/결정성. native 자산 없는 CI는 명시 skip.
  ament_add_pytest_test(wall_hug_runtime test/test_wall_hug_runtime.py TIMEOUT 600
    ENV "OPENBLAS_NUM_THREADS=1" "OMP_NUM_THREADS=1")
  add_executable(tf_owner_observer test/headless/tf_owner_observer.cpp)
  ament_target_dependencies(tf_owner_observer rclcpp tf2_msgs geometry_msgs action_msgs)
  target_link_libraries(tf_owner_observer CycloneDDS::ddsc)
  install(TARGETS tf_owner_observer DESTINATION lib/${PROJECT_NAME})
  add_executable(state_contract_test test/test_state_contract.cpp src/lidar_model.cpp)
  target_include_directories(state_contract_test PRIVATE include "${MUJOCO_INCLUDE_DIR}")
  target_link_libraries(state_contract_test "${MUJOCO_LIBRARY}")
  add_test(NAME state_contract_test COMMAND state_contract_test)
  ament_add_gtest(planar_selection_test
    test/test_planar_selection.cpp
    src/lidar_model.cpp
  )
  target_include_directories(planar_selection_test PRIVATE include "${MUJOCO_INCLUDE_DIR}")
  target_link_libraries(planar_selection_test "${MUJOCO_LIBRARY}")
  # Manual negative proof: same oracle, deliberately altered elevation input.
  # It is not registered with CTest and is never part of the passing suite.
  ament_add_gtest_executable(planar_selection_broken_input
    test/test_planar_selection.cpp src/lidar_model.cpp)
  target_compile_definitions(planar_selection_broken_input PRIVATE W07_BROKEN_INPUT)
  target_include_directories(planar_selection_broken_input PRIVATE include "${MUJOCO_INCLUDE_DIR}")
  target_link_libraries(planar_selection_broken_input "${MUJOCO_LIBRARY}")
  ament_add_pytest_test(stage45_integration_contract test/test_stage45_integration_contract.py)
  ament_add_pytest_test(acceptance_processes test/test_acceptance_processes.py)
  ament_add_pytest_test(acceptance_saved_map test/test_acceptance_saved_map.py)
  ament_add_pytest_test(acceptance_narrow_passage test/test_acceptance_narrow_passage.py)
  ament_add_pytest_test(route_clearance test/test_route_clearance.py)
  ament_add_pytest_test(sim_shutdown test/test_sim_shutdown.py)
  ament_add_pytest_test(sim_session_contract test/test_sim_session.py)
  ament_add_pytest_test(raycast_error_budget test/test_raycast_error_budget.py
    ENV "GO2_LIDAR_EXECUTABLE=$<TARGET_FILE:mujoco_lidar_node>")
  ament_add_pytest_test(stage6_slam_contract test/test_stage6_slam_contract.py
    ENV "GO2_LIDAR_EXECUTABLE=$<TARGET_FILE:mujoco_lidar_node>")
  ament_add_pytest_test(stage62_upright_hold_contract test/test_stage62_upright_hold_contract.py)
endif()

ament_package()
