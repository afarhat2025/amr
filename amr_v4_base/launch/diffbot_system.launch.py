import os, time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction, IncludeLaunchDescription)
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():

    robot_name = os.getenv('ROBOT_MODEL', 'amr_x')
    nav2_bringup = LaunchConfiguration('nav2_bringup')
    manual = LaunchConfiguration('manual')
    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("amr_v4_description"), "urdf", "amr_system.urdf.xacro"]
            ),
        ]
    )
    diffbot_diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("amr_v4_base"),
            "config",
            "diffbot_diff_drive_controller.yaml",
        ]
    )
    navigation_hardware_launch_file_path = PathJoinSubstitution(
        [
            FindPackageShare("amr_v4_navigation"),
            "launch",
            "amr_v4_navigation.launch.py"
        ]
    )
    navigation_launch_file_path = PathJoinSubstitution(
        [
            FindPackageShare("amr_v4_navigation"),
            "launch",
            "nav2_bringup.launch.py"
        ]
    )
    
    mapping_launch_file_path = PathJoinSubstitution(
        [
            FindPackageShare("amr_v4_navigation"),
            "launch",
            "amr_v4_mapping.launch.py"
        ]
    )

    controller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            diffbot_diff_drive_controller,
            {'tf_frame_prefix_enable': True, 'tf_frame_prefix': robot_name}
        ],
        namespace=robot_name,
        remappings=[
            ("~/robot_description", "/" + robot_name + "/robot_description"),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/diagnostics', '/' + robot_name + '/diagnostics')
        ],
    )

    rsp_node = ComposableNode(
        package="robot_state_publisher",
        plugin="robot_state_publisher::RobotStatePublisher",
        namespace=robot_name,
        parameters=[{"robot_description": robot_description_config, 'publish_frequency': 55.0}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    container = ComposableNodeContainer(
        name='rsp',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[rsp_node],
        output='screen',
        #remappings=[('/tf','tf'),('/tf_static','tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')] 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "joint_state_broadcaster"
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "diffbot_base_controller"
        ],
    )
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_navigation_hardware = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_hardware_launch_file_path)
                )
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'nav2_bringup',
        default_value='false',
        choices = ['true','false'],
        description='Enable or disable navigation stack'
    ))
    ld.add_action(DeclareLaunchArgument(
        'manual',
        default_value='false',
        choices = ['true','false'],
        description='false = no localization/mapping, true = localize/mapping launch file active'
    ))
    ld.add_action(controller_node)
    ld.add_action(container)
    ld.add_action(robot_controller_spawner)
    ld.add_action(delay_joint_state_broadcaster)
    #ld.add_action(delay_navigation_hardware)

    ld.add_action(TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_file_path),
                launch_arguments={
                    'mapping': LaunchConfiguration('mapping')
                }.items(),
                condition=IfCondition(manual)
            )
        ]
    ))
    ld.add_action(TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch_file_path),
                condition=IfCondition(nav2_bringup)
            )
        ]
    ))

    return ld