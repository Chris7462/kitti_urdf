from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kitti_urdf"), "launch", "kitti_urdf_launch.py"
            ])
        ])
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", join(get_package_share_directory("kitti_urdf"), "rviz", "kitti_urdf.rviz")]
    )

    return LaunchDescription([
        robot_state_publisher_launch,
        rviz_node
    ])
