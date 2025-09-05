import os
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

description_pkg = "dropbear_granular_urdf"
xacro_filename = "dropbear.urdf.xacro"

def generate_launch_description():
    # Path to xacro file
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', xacro_filename)
    
    # model_arg
    model_args = DeclareLaunchArgument(
        name="model",
        default_value=xacro_file,
        description="Absolute path to robot urdf"
    )

    # Environment Variable for Gazebo model path
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_prefix(description_pkg), "share")

    # robot_description
    robot_description = ParameterValue(Command(
        ['xacro ', LaunchConfiguration("model")]
    ))

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    
    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            "-u"  # This ensures Gazebo starts paused
        ],
        output='screen',
    )
    
    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )
    
    # Spawn entity with delay
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "dropbear",
            "-topic", "/robot_description",
        ],
        output="screen"
    )

    delayed_spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Controller spawners with delays
    controller_spawner_delay = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["stewart_slider_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
            Node(
                package="controller_manager",
                executable="spawner", 
                arguments=["right_hand_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_hand_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["waist_joint_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
        ]
    )

    return LaunchDescription([
        model_args,
        robot_state_publisher,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        delayed_spawn_entity,
        joint_state_broadcaster_spawner,
        controller_spawner_delay,
    ])
