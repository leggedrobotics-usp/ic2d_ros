import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ic2d_description'

    # Robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Set the GAZEBO_MODEL_PATH environment variable to include the models from our own package.
    pkg_install_path = os.path.join(get_package_prefix(package_name), "share")

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + os.pathsep + pkg_install_path
    else:
        model_path = pkg_install_path

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments=[('gz_args', [' -r -v 3 empty.sdf'])]
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'ic2d',
                                   '-allow_renaming', 'false'],
                        output='screen')
    
    # Position controller
    pos_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"]
    )

    # Joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"]
    )

    ros_gz_bridge_config = os.path.join(get_package_share_directory(package_name), "config", "ros_gz_bridge.yaml")

    # ROS GZ bridge (exposes topics for applying joint forces, used to simulate spring/damper)
    ros_gz_bridge = Node(package="ros_gz_bridge",
                         executable="parameter_bridge",
                         ros_arguments=["-p", "config_file:=" + ros_gz_bridge_config])

    # Foxglove bridge (for visualization purposes)
    foxglove_bridge = Node(package='foxglove_bridge',
                           executable='foxglove_bridge')

    # Reference signal generator
    ref_signal_generator = Node(
        package="reference_signal_generator",
        executable="reference_signal_generator",
        ros_arguments=["-p", "topic_name:=/position_controller/commands"]
    )

    # Launch them all!
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        rsp,
        gazebo,
        spawn_entity,
        pos_cont_spawner,
        joint_broad_spawner,
        ref_signal_generator,
        ros_gz_bridge,
        foxglove_bridge
    ])