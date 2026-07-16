cmake_minimum_required(VERSION 3.8)
project(autorunner_ros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ---- 周期反馈/直发消息 (topic) ----
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
  # ---- 请求-响应 service ----
  "srv/EnableJoint.srv"
  "srv/SetJointZero.srv"
  "srv/ClearJointError.srv"
  "srv/SetJointAcc.srv"
  "srv/QueryJointLimit.srv"
  "srv/QueryJointMaxAcc.srv"
  "srv/QueryEndVelAcc.srv"
  "srv/QueryCollisionLevel.srv"
  "srv/SetJointLimit.srv"
  "srv/SetEndVelAcc.srv"
  "srv/SetCollisionLevel.srv"
  "srv/SetMotionCtrl.srv"
  # ---- 运动 action ----
  "action/MoveP.action"
  "action/MoveL.action"
  "action/MoveC.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
