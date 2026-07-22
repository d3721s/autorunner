import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('autorunner_driver'),
        'config', 'u1_arm_driver.yaml')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN 接口名 (仿真调试用 vcan0)')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='u1_arm 参数 yaml 路径')

    # 单进程双节点 (命令节点 u1_arm_driver + 状态节点 u1_udp_publish_node),
    # 由 main.cpp 内部 MultiThreadedExecutor 挂载, 这里只启动可执行文件。
    driver_node = Node(
        package='autorunner_driver',
        executable='u1_arm_driver_node',
        name='u1_arm_driver',
        parameters=[
            LaunchConfiguration('params_file'),
            {'can_interface': LaunchConfiguration('can_interface')},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        can_interface_arg,
        params_file_arg,
        driver_node,
    ])
