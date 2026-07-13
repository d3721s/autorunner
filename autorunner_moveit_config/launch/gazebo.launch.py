import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("autorunner_moveit_config")
    description_share = get_package_share_directory("autorunner_description")

    # Let Gazebo resolve package:// mesh URIs from autorunner_description
    set_resource_path = AppendEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH", os.path.dirname(description_share)
    )

    # --- Robot description (Gazebo version: ign_ros2_control hardware plugin) ---
    gazebo_xacro = os.path.join(pkg_share, "config", "autorunner_gazebo.urdf.xacro")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", gazebo_xacro]),
        value_type=str,
    )

    # --- MoveIt config (SRDF, kinematics, planning, etc.) ---
    moveit_config = (
        MoveItConfigsBuilder("autorunner", package_name="autorunner_moveit_config")
        .robot_description(file_path="config/autorunner_gazebo.urdf.xacro")
        .to_moveit_configs()
    )

    # --- Gazebo (Fortress / ign) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-r -v 3 empty.sdf"}.items(),
    )

    # Bridge Gazebo sim clock -> ROS /clock so everything runs on sim time
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # Spawn the robot into Gazebo from /robot_description
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "autorunner"],
        output="screen",
    )

    # --- Controllers (controller_manager runs inside Gazebo via the plugin) ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["autorunnerbase_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --- MoveIt move_group ---
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # --- RViz ---
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            set_resource_path,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_entity,
            # Start controllers only after the robot has been spawned in Gazebo
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner, move_group, rviz],
                )
            ),
        ]
    )
