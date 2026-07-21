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
  "msg/autorunner/Armpose.msg"
  "msg/autorunner/Armstatus.msg"
  "msg/autorunner/Jointangle.msg"
  "msg/autorunner/Jointspeed.msg"
  "msg/autorunner/Jointcurrent.msg"
  "msg/autorunner/Jointmotorpos.msg"
  "msg/autorunner/Jointvoltage.msg"
  "msg/autorunner/Jointtemperature.msg"
  "msg/autorunner/Jointerrorcode.msg"
  "msg/autorunner/Jointlimit.msg"
  "msg/autorunner/Jointmaxacc.msg"
  "msg/autorunner/Setresponse.msg"
  "msg/autorunner/Movej.msg"
  "msg/autorunner/Movep.msg"
  "msg/autorunner/Movec.msg"
  "msg/autorunner/Stop.msg"
  "msg/autorunner/Motionctrl.msg"
  "msg/autorunner/Modectrl.msg"
  "msg/autorunner/Jointenable.msg"
  "msg/autorunner/Jointconfig.msg"
  "msg/autorunner/Jointlimitset.msg"
  "msg/autorunner/Jointlimitquery.msg"
  "msg/autorunner/Collisionlevel.msg"
  "msg/autorunner/Endvelacc.msg"
  "msg/autorunner/Jointmit.msg"
  # ---- 请求-响应 service ----
  "srv/autorunner/EnableJoint.srv"
  "srv/autorunner/SetJointZero.srv"
  "srv/autorunner/ClearJointError.srv"
  "srv/autorunner/SetJointAcc.srv"
  "srv/autorunner/QueryJointLimit.srv"
  "srv/autorunner/QueryJointMaxAcc.srv"
  "srv/autorunner/QueryEndVelAcc.srv"
  "srv/autorunner/QueryCollisionLevel.srv"
  "srv/autorunner/SetJointLimit.srv"
  "srv/autorunner/SetEndVelAcc.srv"
  "srv/autorunner/SetCollisionLevel.srv"
  "srv/autorunner/SetMotionCtrl.srv"
  # ---- 运动 action ----
  "action/autorunner/MoveP.action"
  "action/autorunner/MoveL.action"
  "action/autorunner/MoveC.action"
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
