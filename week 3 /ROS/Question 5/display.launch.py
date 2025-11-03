import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the path to the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_robot_pkg'))
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'my_robot.urdf')

    # Load the URDF content
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = robot_description_config.toxml()

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # Joint State Publisher GUI Node
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 Node
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('my_robot_pkg'), 'config', 'rviz_config.rviz')]
    )
    
    # We need to create a default rviz config file
    # Or we can launch rviz2 without a config file
    node_rviz2_simple = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # Note: You'll have to manually add displays in RViz
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2_simple  # Use this simple version first
    ])
