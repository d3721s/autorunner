cmake_minimum_required(VERSION 3.8)
project(autorunner_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(autorunner_ros_interfaces REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# u1_arm 额外依赖
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 协议层静态库：纯 C++，零 ROS 依赖，节点与 gtest 共用
add_library(autorunner_protocol STATIC
  src/autorunner_driver/protocol/encoder.cpp
  src/autorunner_driver/protocol/decoder.cpp
  src/autorunner_driver/protocol/assembler.cpp
)
target_include_directories(autorunner_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(autorunner_protocol PUBLIC cxx_std_17)

# ---- u1_arm (5关节达妙电机机械臂) ----
# 达妙电机 CAN 协议层静态库：纯 C++，零 ROS 依赖
add_library(u1_damiao_protocol STATIC
  src/u1_arm_driver/protocol/damiao_protocol.cpp
)
target_include_directories(u1_damiao_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(u1_damiao_protocol PUBLIC cxx_std_17)

# 驱动节点
add_executable(autorunner_driver_node
  src/autorunner_driver/driver_node.cpp
  src/autorunner_driver/driver_node_services.cpp
  src/autorunner_driver/driver_node_actions.cpp
  src/autorunner_driver/main.cpp
)
target_link_libraries(autorunner_driver_node autorunner_protocol)
ament_target_dependencies(autorunner_driver_node
  rclcpp
  rclcpp_action
  autorunner_ros_interfaces
  ros2_socketcan
  can_msgs
  sensor_msgs
  std_msgs
  std_srvs
  control_msgs
  trajectory_msgs
)

# u1_arm 驱动节点 (命令节点 + 状态发布节点, 单进程)
add_executable(u1_arm_driver_node
  src/u1_arm_driver/can_bus.cpp
  src/u1_arm_driver/motor_manager.cpp
  src/u1_arm_driver/kinematics.cpp
  src/u1_arm_driver/trajectory_executor.cpp
  src/u1_arm_driver/u1_arm_node.cpp
  src/u1_arm_driver/u1_arm_node_query.cpp
  src/u1_arm_driver/u1_arm_node_stubs.cpp
  src/u1_arm_driver/state_publisher_node.cpp
  src/u1_arm_driver/main.cpp
)
target_link_libraries(u1_arm_driver_node u1_damiao_protocol)
ament_target_dependencies(u1_arm_driver_node
  rclcpp
  autorunner_ros_interfaces
  ros2_socketcan
  sensor_msgs
  std_msgs
  geometry_msgs
  kdl_parser
  orocos_kdl
  ament_index_cpp
)

install(TARGETS autorunner_driver_node u1_arm_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)
# u1_arm 工具脚本 (仿真器 / 电机标定 CLI)
install(PROGRAMS
  tools/u1_arm/fake_motors.py
  tools/u1_arm/motor_setup.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol
    test/autorunner_driver/test_endian.cpp
    test/autorunner_driver/test_encoder.cpp
    test/autorunner_driver/test_decoder.cpp
    test/autorunner_driver/test_assembler.cpp
  )
  target_link_libraries(test_protocol autorunner_protocol)

  ament_add_gtest(test_u1_damiao_protocol
    test/u1_arm_driver/test_damiao_protocol.cpp
  )
  target_link_libraries(test_u1_damiao_protocol u1_damiao_protocol)
endif()

ament_package()
