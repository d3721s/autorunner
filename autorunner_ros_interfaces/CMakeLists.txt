cmake_minimum_required(VERSION 3.8)
project(autorunner_ros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armpose.msg"
  "msg/Armstatus.msg"
  "msg/Jointangle.msg"
  "msg/Jointspeed.msg"
  "msg/Jointcurrent.msg"
  "msg/Jointmotorpos.msg"
  "msg/Jointvoltage.msg"
  "msg/Jointtemperature.msg"
  "msg/Jointerrorcode.msg"
  "msg/Jointlimit.msg"
  "msg/Jointmaxacc.msg"
  "msg/Setresponse.msg"
  "msg/Movej.msg"
  "msg/Movep.msg"
  "msg/Movec.msg"
  "msg/Stop.msg"
  "msg/Motionctrl.msg"
  "msg/Modectrl.msg"
  "msg/Jointenable.msg"
  "msg/Jointconfig.msg"
  "msg/Jointlimitset.msg"
  "msg/Jointlimitquery.msg"
  "msg/Collisionlevel.msg"
  "msg/Endvelacc.msg"
  "msg/Jointmit.msg"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
