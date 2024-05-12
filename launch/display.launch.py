import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
import launch_ros
import launch
import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('cobalt_simulation'))
    xacro_file = os.path.join(pkg_path, 'description', 'urdf', 'urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Default RViz config path
    default_rviz_config_path = os.path.join(pkg_path, 'rviz', 'rviz_config.rviz')

    # Create a RViz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    # Launch all
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false', 
            description='Use sim time if true'
        ),
        node_robot_state_publisher,
        rviz_node
    ])
