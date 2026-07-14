import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('autorunner_driver'),
        'config', 'autorunner_driver.yaml')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN 接口名 (调试用 vcan0)')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='参数 yaml 路径')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='节点命名空间')

    driver_node = Node(
        package='autorunner_driver',
        executable='autorunner_driver_node',
        name='autorunner_driver',
        namespace=LaunchConfiguration('namespace'),
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
        namespace_arg,
        driver_node,
    ])
