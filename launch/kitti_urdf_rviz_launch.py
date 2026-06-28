from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kitti_urdf')

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(pkg_share, 'launch', 'kitti_urdf_launch.py')
        ])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(pkg_share, 'rviz', 'kitti_urdf.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_launch,
        rviz_node
    ])
