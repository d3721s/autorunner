# 真机 launch: MoveIt 通过 autorunner_driver 走 CAN 控制真实机械臂。
#
# 与 demo.launch.py 的区别:
#   demo 会启动 ros2_control_node + mock 硬件 + autorunnerbase_controller (仿真占位),
#   那个占位控制器和本驱动争抢同一个 follow_joint_trajectory action 名, 会打架。
#   真机 launch 只启动 MoveIt 规划/显示部分, 由 autorunner_driver 提供该 action 和
#   /joint_states, 不启动任何 ros2_control 控制器。
#
# 前置: 先(或由本 launch)启动 autorunner_driver, 其 move_joint action 名默认已对齐
#   /autorunnerbase_controller/follow_joint_trajectory (见驱动参数 move_joint_action_name)。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("autorunner", package_name="autorunner_moveit_config")
        .to_moveit_configs()
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="can1",
        description="SocketCAN 接口名 (真机默认 can1, 仿真调试可用 vcan0)")
    launch_driver_arg = DeclareLaunchArgument(
        "launch_driver", default_value="true",
        description="是否由本 launch 一并启动 autorunner_driver (false 则需另行启动)")
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true",
        description="是否启动 RViz")

    ld = LaunchDescription()
    ld.add_action(can_interface_arg)
    ld.add_action(launch_driver_arg)
    ld.add_action(launch_rviz_arg)

    # ---- CAN 驱动: 提供 follow_joint_trajectory action + /joint_states ----
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("autorunner_driver"),
                "launch", "autorunner_driver.launch.py",
            ])
        ),
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )
    ld.add_action(driver_launch)

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
