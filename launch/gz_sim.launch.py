import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    undeformed_length = LaunchConfiguration('undeformed_length')
    stiffness = LaunchConfiguration('stiffness')
    damping = LaunchConfiguration('damping')    

    package_name='ic2d_description'

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Set the GAZEBO_MODEL_PATH environment variable to include the models from our own package.
    pkg_install_path = os.path.join(get_package_prefix(package_name), "share")

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + os.pathsep + pkg_install_path
    else:
        model_path = pkg_install_path

    gz_world_path = os.path.join(get_package_share_directory(package_name), "world", "ic2d_empty.sdf")

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # Verbosity level: 1 (errors only)
    # Server only (headless) mode: "-s" argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', ['-s -r -v 1 ' + gz_world_path])])

    # Run the spawner node from the gazebo_ros package.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'ic2d',
                                   '-allow_renaming', 'false'],
                        output='screen')
    
    # Position controller
    pos_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"])

    # Joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"])

    ros_gz_bridge_config = os.path.join(get_package_share_directory(package_name), "config", "ros_gz_bridge.yaml")

    # ROS GZ bridge (exposes topics for applying joint forces, used to simulate spring/damper)
    ros_gz_bridge = Node(package="ros_gz_bridge",
                         executable="parameter_bridge",
                         ros_arguments=["-p", "config_file:=" + ros_gz_bridge_config])

    # Reference signal generator
    ref_signal_generator = Node(
        package="reference_signal_generator",
        executable="reference_signal_generator",
        ros_arguments=["-p", "topic_name:=/position_controller/commands"])

    # Virtual spring/damper
    virtual_spring_damper = Node(package="ic2d_description",
                                 executable="virtual_spring_damper",
                                 ros_arguments=["-p", ["undeformed_length:=", undeformed_length],
                                                "-p", ["stiffness:=", stiffness],
                                                "-p", ["damping:=", damping]])
    
    # Gazebo Real Time Factor (RTF) publisher
    rtf_publisher = Node(package="gz_rtf_publisher",
                         executable="gz_rtf_publisher")

    # Launch them all!
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        DeclareLaunchArgument(
            'undeformed_length',
            default_value='0.3',
            description='Undeformed length of the spring'
        ),
        DeclareLaunchArgument(
           'stiffness',
            default_value='6000.0',
            description='Stiffness of the spring'
        ),
        DeclareLaunchArgument(
            'damping',
            default_value='100.0',
            description='Damping coefficient of the damper'
        ),
        rsp,
        gazebo,
        spawn_entity,
        pos_cont_spawner,
        joint_broad_spawner,
        ref_signal_generator,
        ros_gz_bridge,
        virtual_spring_damper,
        rtf_publisher
    ])