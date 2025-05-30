import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

def on_matching_output(matcher: str, result):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result
    return on_output

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default='amr_x')

    # Define messages to be matched
    diff_drive_loaded_message = f"Loaded {robot_name}/diffbot_base_controller"
    ekf_node_started_message = "EKF node started"

    # Launch the diffbot_system_launch.py file
    diffbot_system_launch = ExecuteProcess(
        name="launch_diffbot_system",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("amr_v4_base"),
                    "launch",
                    "diffbot_system.launch.py"
                ]
            )
        ],
        output="screen"
    )

    # Event handler for diffbot_system_launch to wait for specific output
    waiting_for_diff_drive = RegisterEventHandler(
        OnProcessIO(
            target_action=diffbot_system_launch,
            on_stdout=on_matching_output(
                diff_drive_loaded_message,
                [
                    LogInfo(
                        msg="Diff drive controller loaded. Preparing to start navigation..."
                    ),
                ]
            )
        )
    )

    # Launch the amr_v4_navigation.launch.py file
    navigation_launch = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("amr_v4_navigation"),
                    "launch",
                    "amr_v4_navigation.launch.py"
                ]
            )
        ],
        output="screen"
    )

    # Event handler for navigation launch to wait for specific output
    waiting_for_navigation = RegisterEventHandler(
        OnProcessIO(
            target_action=navigation_launch,
            on_stdout=on_matching_output(
                ekf_node_started_message,
                [
                    LogInfo(
                        msg="Navigation stack started successfully."
                    ),
                ]
            )
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="amr_x",
                description="Name of the robot."
            ),
            diffbot_system_launch,
            waiting_for_diff_drive,
            navigation_launch,
            waiting_for_navigation,
        ]
    )
