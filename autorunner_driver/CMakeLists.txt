cmake_minimum_required(VERSION 3.8)
project(autorunner_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autorunner_ros_interfaces REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 协议层静态库：纯 C++，零 ROS 依赖，节点与 gtest 共用
add_library(autorunner_protocol STATIC
  src/protocol/encoder.cpp
  src/protocol/decoder.cpp
  src/protocol/assembler.cpp
)
target_include_directories(autorunner_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(autorunner_protocol PUBLIC cxx_std_17)

# 驱动节点
add_executable(autorunner_driver_node
  src/driver_node.cpp
  src/main.cpp
)
target_link_libraries(autorunner_driver_node autorunner_protocol)
ament_target_dependencies(autorunner_driver_node
  rclcpp
  autorunner_ros_interfaces
  ros2_socketcan
  can_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS autorunner_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol
    test/test_endian.cpp
    test/test_encoder.cpp
    test/test_decoder.cpp
    test/test_assembler.cpp
  )
  target_link_libraries(test_protocol autorunner_protocol)
endif()

ament_package()
