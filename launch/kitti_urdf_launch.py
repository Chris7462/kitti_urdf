from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
import xacro


def generate_launch_description():
    pkg_name = "kitti_urdf"
    xacro_file = "description/kitti_urdf.xacro"
    rviz_file = "rviz/kitti_urdf.rviz"

    # Use xacro to process the file
    xacro_file = join(get_package_share_directory(pkg_name), xacro_file)
    kitti_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": kitti_description_raw}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", join(get_package_share_directory(pkg_name), rviz_file)]
    )

    # Run the node
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
