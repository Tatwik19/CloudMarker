import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path = os.path.expanduser('~/ros2_ws/bag/ass1/rgbd_bag_0')

    pkg_share = get_package_share_directory('ass1_perception')
    rviz_config = os.path.join(pkg_share, 'rviz', 'ass1_perception.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    cylinder_detector = Node(
        package='ass1_perception',
        executable='cylinder_detector',
        name='cylinder_detector',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen',
    )

    rqt_image_view = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
            '--clear-config',
            '--ros-args', '-p', 'image_transport:=compressed',
        ],
        output='screen',
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--loop', '--rate', '0.5'],
        output='screen',
    )

    return LaunchDescription([
        cylinder_detector,
        rviz,
        rqt_image_view,
        TimerAction(period=2.0, actions=[bag_play]),
    ])