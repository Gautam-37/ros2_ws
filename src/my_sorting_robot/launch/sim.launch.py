import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(get_package_share_directory('my_sorting_robot'), 'worlds', 'sorting_world.sdf')

    # -r run immediately, -s server only (no GUI = far lighter on a VM). Software rendering for VMware.
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen')

    vision = Node(package='my_sorting_robot', executable='vision_node', output='screen')

    return LaunchDescription([gz, bridge, vision])
