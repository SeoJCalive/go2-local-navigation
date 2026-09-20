cmake_minimum_required(VERSION 3.16)
project(go2_mujoco_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)

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
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/scene/go2_indoor.xml"
  DESTINATION share/${PROJECT_NAME}/scene)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(stage45_integration_contract test/test_stage45_integration_contract.py)
  ament_add_pytest_test(stage6_slam_contract test/test_stage6_slam_contract.py)
  ament_add_pytest_test(stage62_upright_hold_contract test/test_stage62_upright_hold_contract.py)
endif()

ament_package()
