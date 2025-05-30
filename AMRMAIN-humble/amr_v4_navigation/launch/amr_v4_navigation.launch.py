#!/usr/bin/python -u
from email.policy import default
import os, re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument,TimerAction,RegisterEventHandler, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    config_file_common_path = os.path.join(get_package_share_directory('amr_v4_navigation'),'config')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    robot_name = (os.getenv('ROBOT_MODEL','amr_x'))
    #config_camera_path = os.path.join(get_package_share_directory('zed_wrapper'),'config')
    camera_model = "zedx"
    declare_joy_config = DeclareLaunchArgument('joy_config', default_value='ps3')
    declare_joy_dev = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    declare_config_filepath = DeclareLaunchArgument('config_filepath', default_value=[
        TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
        joy_config, TextSubstitution(text='.config.yaml')])
    

    joy_node = Node(
            package='joy', executable='joy_node', name='joy_node',
            namespace=robot_name,
            parameters=[{
                'device_id': 0,
                'dev': joy_dev,
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            }])
        
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', 
            namespace=robot_name,
            parameters=[config_file_common_path + '/params.yaml'],
            remappings={('/'+robot_name+'/cmd_vel', '/'+robot_name+'/joy_cmd_vel')},
            )

    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node', 
            namespace=robot_name,
            remappings={('/'+robot_name+'/cmd_vel', '/'+robot_name+'/key_cmd_vel')},
            output='screen',
            prefix = 'xterm -e',
            )

    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            namespace=robot_name,
            output='screen',
            parameters=[config_file_common_path + '/params.yaml'],
            remappings=[('/'+robot_name+'/cmd_vel_out','/'+robot_name+'/diffbot_base_controller/cmd_vel_unstamped'),
                        ('/diagnostics','/'+robot_name+'/diagnostics')]
        )
    
    # zed_wrapper_node = Node(
    #     package='zed_components',
    #     namespace= robot_name,
    #     executable='zed_wrapper',
    #     name='zed_node',
    #     output='screen',
    #     parameters=[
    #         config_file_common_path + '/common_stereo.yaml',
    #         config_file_common_path + '/' + camera_model + '.yaml',
    #     ],
    #     remappings=[('/tf','tf'),('/tf_static','tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')] 
    # )
    zed_wrapper_node = ComposableNode(
        package='zed_components',
        namespace= robot_name,
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            config_file_common_path + '/common_stereo.yaml',
            config_file_common_path + '/' + camera_model + '.yaml',
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
        remappings=[('/tf','/'+robot_name+'/tf'),('/tf_static','/'+robot_name+'/tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')] 
        
    )
    container = ComposableNodeContainer(
        name='zed_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[zed_wrapper_node],
        output='screen',
        remappings=[('/tf','tf'),('/tf_static','tf_static'),('/diagnostics','/'+robot_name+'/diagnostics')] 
    )

    

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_name,
        output='screen',
        remappings={('/'+robot_name+'/odometry/filtered', '/'+robot_name+'/odom/ekf'),('/tf','tf'),('/tf_static','tf_static'),
                     ('/diagnostics','/'+robot_name+'/diagnostics')},
        parameters=[config_file_common_path + '/ekf.yaml',{'tf_prefix':robot_name}],
        
    )

    hesai_lidar = Node(
        namespace=robot_name, 
        package='hesai_ros_driver', 
        executable='hesai_ros_driver_node', 
        output='screen',
        remappings=[('hesai_points','/'+robot_name+'/hesai_points')]
        )

    obstacle_lidar = Node(
    package='sick_scan_xd',
    executable='sick_generic_caller',
    namespace=robot_name,
    output='screen',
    parameters=[{
        'auto_reboot': True,
        'scanner_type': 'sick_tim_7xx',
        'hostname': '192.168.0.1',
        'nodename': 'sick',
        'min_ang': -0.78,  # Corrected value from your original code
        'max_ang': 0.78,   # Corrected value from your original code
        'use_binary_protocol': True,
        'range_min': 0.0,
        'range_max': 100.0,
        'range_filter_handling': 0,
        'intensity': False,
        'laserscan_topic': 'scan_obstacle',
        'frame_id': robot_name + '/nav2_obstacle',
        'port': '2112',
        'range_filter_handling': 0,
        'field_set_selection_method': -1,
        'active_field_set': -1,
        'scandatacfg_timingflag': -1,
        'timelimit': 5,
        'sw_pll_only_publish': False,
        'use_generation_timestamp': True,
        'start_services': True,
        'activate_lferec': True,
        'activate_lidoutputstate': True,
        'activate_lidinputstate': False,
        'add_transform_check_dynamic_updates': False,
        'message_monitoring_enabled': True,
        'ros_qos': 4,  # Default QoS value for ROS 2
        'time_offset': -0.47,  # Time offset -0.47
        'skip': 1,
        'tf_base_frame_id': robot_name + '/base_link',
        'tf_base_lidar_xyz_rpy': '0.461,-0.203,0.136,0,0,0',
        'tf_publish_rate': 50.0,
        'time_increment': 0.0,
        'add_transform_xyz_rpy': '0,0,0,0,0,0',  # No transform by default
        'add_transform_check_dynamic_updates': False,
        'tick_to_timestamp_mode': 0,
    }],
    remappings=[('/diagnostics', '/' + robot_name + '/diagnostics'),
                ('/cloud', '/' + robot_name + '/cloud_not_used'),
                ('/' + robot_name + '/imu', '/' + robot_name + '/imu_unused'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    )

    gui = Node(
            package='amr_v4_qt',
            executable='amr_v4_qt',
            output='screen',
            namespace=robot_name,
            remappings=[('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
        )
    charger_bms = Node(
            package='amr_v4_navigation',
            executable='charging',
            output='screen'
        )


    start_hesai_lidar = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=container,
            on_stdout=lambda event: [
                LogInfo(msg='Hesai Lidar output: "{}"'.format(event.text.decode().strip())),
                hesai_lidar
            ] if "Serial Number: S/N" in event.text.decode() else None
        )
    )

    start_obstacle_lidar = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=hesai_lidar,
            on_stdout=lambda event: [
                LogInfo(msg='Hesai Lidar output: "{}"'.format(event.text.decode().strip())),
                obstacle_lidar
            ] if "SocketSource::Open succeed" in event.text.decode() else None
        )
    )

    return LaunchDescription([
        declare_joy_config,
        declare_joy_dev,
        declare_config_filepath,
        container,
        start_hesai_lidar,
        start_obstacle_lidar,
        twist_mux_node,
        robot_localization_node,
        joy_node,
        teleop_twist_joy_node,
        #teleop_twist_keyboard_node,
        #gui,
        charger_bms

    ])

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()