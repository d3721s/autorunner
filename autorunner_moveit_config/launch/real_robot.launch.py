# 真机 launch: 只启动 MoveIt 规划/显示部分 (move_group + rsp + 静态TF + RViz)。
# 机械臂驱动 autorunner_driver 始终单独启动, 由它提供 /joint_states 和
# follow_joint_trajectory action; 本 launch 不启动驱动、也不启动任何 ros2_control 控制器。
#
# 分离启动流程:
#   终端1:  ros2 launch autorunner_driver autorunner_driver.launch.py can_interface:=can1
#   终端2:  确认 /joint_states 有真实关节角:  ros2 topic echo /joint_states
#   终端3:  ros2 launch autorunner_moveit_config real_robot.launch.py
# 必须先确认驱动的 /joint_states 出真实关节角再启动 MoveIt, 否则 MoveIt 用默认全零位,
# 会在虚构姿态上误判碰撞, 导致规划失败。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("autorunner", package_name="autorunner_moveit_config")
        .to_moveit_configs()
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true",
        description="是否启动 RViz")

    ld = LaunchDescription()
    ld.add_action(launch_rviz_arg)

    # ---- robot_state_publisher (URDF -> TF) ----
    for entity in generate_rsp_launch(moveit_config).entities:
        ld.add_action(entity)

    # ---- 静态虚拟关节 TF (base <-> world) ----
    for entity in generate_static_virtual_joint_tfs_launch(moveit_config).entities:
        ld.add_action(entity)

    # ---- move_group (MoveIt 规划核心) ----
    for entity in generate_move_group_launch(moveit_config).entities:
        ld.add_action(entity)

    # ---- RViz (MotionPlanning 插件); 复用 MoveIt 生成器, 由 launch_rviz 开关 ----
    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("autorunner_moveit_config"),
                "launch", "moveit_rviz.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    ld.add_action(rviz_include)

    return ld
