import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    joint_1_config = LaunchConfiguration('joint_1_config')
    joint_2_config = LaunchConfiguration('joint_2_config')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('ic2d_description'))
    xacro_file = os.path.join(pkg_path,'description','ic2d.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = Command(["xacro ", xacro_file, " joint_1_config:=", joint_1_config, " joint_2_config:=", joint_2_config])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Foxglobe bridge (for visualization purposes)
    foxglove_bridge = Node(package='foxglove_bridge', executable='foxglove_bridge')

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'joint_1_config',
            default_value='linmot',
            description='Joint 1 configuration. Possible values: "linmot", "hydraulic", "fixed"'
        ),
        DeclareLaunchArgument(
            'joint_2_config',
            default_value='linmot',
            description='Joint 2 configuration. Possible values: "linmot", "hydraulic", "fixed"'
        ),
        robot_state_publisher,
        foxglove_bridge
    ])
