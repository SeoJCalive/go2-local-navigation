cmake_minimum_required(VERSION 3.16)
project(go2_counterfactual_replay)
# Standalone build in an isolated prefix; never part of colcon/product install.
# 목적/흐름: 선택적 RUNTIME_PREFIX로 설치 SDK/configure를 연결한다.
# 입출력: pinned source·overlay prefix → replay executable. offline solver는 worker1.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
if(NOT EXISTS "${ENGINE_SOURCE}/lib/karto_sdk/src/Mapper.cpp")
  message(FATAL_ERROR "ENGINE_SOURCE must be the pinned, patched slam_toolbox source")
endif()
find_package(ament_cmake REQUIRED)
set(dependencies rclcpp tf2_ros tf2_geometry_msgs sensor_msgs nav_msgs
  geometry_msgs visualization_msgs std_msgs std_srvs pluginlib
  interactive_markers slam_toolbox)
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(TBB REQUIRED)
if(RUNTIME_PREFIX)
  set(mapper_library "${RUNTIME_PREFIX}/lib/libtoolbox_common.so")
  set(karto_library "${RUNTIME_PREFIX}/lib/libkartoSlamToolbox.so")
  add_executable(engine_replay engine_replay.cpp
    "${ENGINE_SOURCE}/solvers/ceres_solver.cpp")
  target_compile_definitions(engine_replay PRIVATE GO2_RUNTIME_REPLAY)
  target_include_directories(engine_replay BEFORE PRIVATE
    "${RUNTIME_SOURCE}/lib/karto_sdk/include" "${RUNTIME_SOURCE}/include")
else()
  add_subdirectory("${ENGINE_SOURCE}/lib/karto_sdk" karto)
  set(karto_library kartoSlamToolbox)
  set(mapper_library "")
  add_executable(engine_replay engine_replay.cpp
    "${ENGINE_SOURCE}/src/slam_mapper.cpp"
    "${ENGINE_SOURCE}/solvers/ceres_solver.cpp")
endif()
target_include_directories(engine_replay PRIVATE
  "${ENGINE_SOURCE}" "${ENGINE_SOURCE}/include"
  "${ENGINE_SOURCE}/lib/karto_sdk/include" ${CERES_INCLUDE_DIRS})
ament_target_dependencies(engine_replay ${dependencies})
target_link_libraries(engine_replay ${mapper_library} ${karto_library} ${CERES_LIBRARIES}
  ${Boost_LIBRARIES} TBB::tbb)
target_link_options(engine_replay PRIVATE -rdynamic -Wl,--disable-new-dtags)
if(RUNTIME_PREFIX)
  set_target_properties(engine_replay PROPERTIES BUILD_RPATH "${RUNTIME_PREFIX}/lib")
else()
  set_target_properties(engine_replay PROPERTIES BUILD_RPATH "${CMAKE_BINARY_DIR}/karto")
endif()
