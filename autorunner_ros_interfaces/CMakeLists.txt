cmake_minimum_required(VERSION 3.8)
project(autorunner_ros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---- AutoRunner (PiperX 底盘臂) 接口 ----
set(AUTORUNNER_INTERFACES
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
)

# ---- u1_arm (5关节达妙电机机械臂) 接口: 镜像 rm_driver 对外话题所用消息 ----
# 注意: rosidl 会拍平子目录, 与 autorunner 组重名的 8 个消息加 U1 前缀
# (U1Movej/U1Movec/U1Stop/U1Jointerrorcode/U1Jointcurrent/U1Jointspeed/
#  U1Jointtemperature/U1Jointvoltage), 其余保持 rm_ros_interfaces 原名/原字段。
set(U1_ARM_INTERFACES
  # 运动命令
  "msg/u1_arm/U1Movej.msg"
  "msg/u1_arm/Movejp.msg"
  "msg/u1_arm/Movel.msg"
  "msg/u1_arm/U1Movec.msg"
  "msg/u1_arm/Moveloffset.msg"
  "msg/u1_arm/Jointpos.msg"
  "msg/u1_arm/Jointposcustom.msg"
  "msg/u1_arm/Cartepos.msg"
  "msg/u1_arm/Carteposcustom.msg"
  "msg/u1_arm/U1Stop.msg"
  # 示教
  "msg/u1_arm/Jointteach.msg"
  "msg/u1_arm/Posteach.msg"
  "msg/u1_arm/Ortteach.msg"
  # 状态/查询
  "msg/u1_arm/Setrealtimepush.msg"
  "msg/u1_arm/Armsoftversion.msg"
  "msg/u1_arm/Softwarebuildinfo.msg"
  "msg/u1_arm/Sixforce.msg"
  "msg/u1_arm/U1Jointerrorcode.msg"
  "msg/u1_arm/Armoriginalstate.msg"
  "msg/u1_arm/Armstate.msg"
  "msg/u1_arm/Armcurrentstatus.msg"
  "msg/u1_arm/U1Jointcurrent.msg"
  "msg/u1_arm/Jointenflag.msg"
  "msg/u1_arm/Jointposeeuler.msg"
  "msg/u1_arm/U1Jointspeed.msg"
  "msg/u1_arm/U1Jointtemperature.msg"
  "msg/u1_arm/U1Jointvoltage.msg"
  "msg/u1_arm/Rmerr.msg"
  "msg/u1_arm/RobotInfo.msg"
  "msg/u1_arm/Jointversion.msg"
  "msg/u1_arm/Toolsoftwareversionv4.msg"
  # 力位混合
  "msg/u1_arm/Forcepositionmove.msg"
  "msg/u1_arm/Forcepositionmovejoint.msg"
  "msg/u1_arm/Forcepositionmovepose.msg"
  "msg/u1_arm/Setforceposition.msg"
  # 坐标系
  "msg/u1_arm/Getallframe.msg"
  # 关节维护
  "msg/u1_arm/Jointerrclear.msg"
  # 夹爪/灵巧手
  "msg/u1_arm/Gripperpick.msg"
  "msg/u1_arm/Gripperset.msg"
  "msg/u1_arm/Handangle.msg"
  "msg/u1_arm/Handforce.msg"
  "msg/u1_arm/Handposture.msg"
  "msg/u1_arm/Handseq.msg"
  "msg/u1_arm/Handspeed.msg"
  "msg/u1_arm/Handstatus.msg"
  # 升降/扩展关节
  "msg/u1_arm/Liftspeed.msg"
  "msg/u1_arm/Liftheight.msg"
  "msg/u1_arm/Liftstate.msg"
  "msg/u1_arm/Udpliftstate.msg"
  "msg/u1_arm/Expandstate.msg"
  "msg/u1_arm/Expandpos.msg"
  "msg/u1_arm/Udpexpandstate.msg"
  # 末端生态 (Rmplus)
  "msg/u1_arm/Rmplusbase.msg"
  "msg/u1_arm/Rmplusstate.msg"
  # 在线编程/轨迹文件/Modbus (四代控制器组, 打桩用)
  "msg/u1_arm/Flowchartrunstate.msg"
  "msg/u1_arm/Trajectoryinfo.msg"
  "msg/u1_arm/Trajectorylist.msg"
  "msg/u1_arm/Gettrajectorylist.msg"
  "msg/u1_arm/Modbustcpmasterinfo.msg"
  "msg/u1_arm/Modbustcpmasterupdata.msg"
  "msg/u1_arm/Modbustcpmasterlist.msg"
  "msg/u1_arm/Modbustcpreadparams.msg"
  "msg/u1_arm/Modbustcpwriteparams.msg"
  "msg/u1_arm/Modbusrtureadparams.msg"
  "msg/u1_arm/Modbusrtuwriteparams.msg"
  "msg/u1_arm/Modbusreaddata.msg"
  "msg/u1_arm/Mastername.msg"
  "msg/u1_arm/Getmodbustcpmasterlist.msg"
  "msg/u1_arm/RS485params.msg"
  "msg/u1_arm/Programrunstate.msg"
  "msg/u1_arm/Sendproject.msg"
  "msg/u1_arm/Alohastate.msg"
)

# rosidl_generate_interfaces 每包仅允许调用一次, 两组接口在此合并生成
rosidl_generate_interfaces(${PROJECT_NAME}
  ${AUTORUNNER_INTERFACES}
  ${U1_ARM_INTERFACES}
  DEPENDENCIES action_msgs std_msgs geometry_msgs builtin_interfaces
)

# rosidl 按源相对路径安装原始接口文件(share/<pkg>/autorunner|u1_arm/), 而
# `ros2 interface show` 只查找 share/<pkg>/{msg,srv,action}/ 平铺路径,
# 这里补一份平铺安装(文件名两组互不重名, 已由命名约定保证)
install(DIRECTORY msg/autorunner/ msg/u1_arm/ DESTINATION share/${PROJECT_NAME}/msg)
install(DIRECTORY srv/autorunner/ DESTINATION share/${PROJECT_NAME}/srv)
install(DIRECTORY action/autorunner/ DESTINATION share/${PROJECT_NAME}/action)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
