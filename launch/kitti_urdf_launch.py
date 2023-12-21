from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
import xacro


def generate_launch_description():
    # Use xacro to process the file
    xacro_file = join(get_package_share_directory("kitti_urdf"), "description", "kitti_urdf.xacro")
    kitti_description = xacro.process_file(xacro_file).toxml()

    # Configure the node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": kitti_description
        }]
    )

    # Run the node
    return LaunchDescription([
        robot_state_publisher_node
    ])
