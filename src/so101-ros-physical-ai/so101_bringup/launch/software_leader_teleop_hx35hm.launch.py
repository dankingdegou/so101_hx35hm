from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    follower_ns = LaunchConfiguration("follower_namespace")
    follower_device = LaunchConfiguration("follower_device")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    follower_params_file = LaunchConfiguration("follower_params_file")
    teleop_params_file = LaunchConfiguration("teleop_params_file")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")
    use_gui = LaunchConfiguration("use_gui")
    use_teleop = LaunchConfiguration("use_teleop")
    launch_follower = LaunchConfiguration("launch_follower")

    default_follower_params = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "hx35hm_follower_bridge_params.yaml"]
    )
    default_teleop_params = PathJoinSubstitution(
        [FindPackageShare("so101_teleop"), "config", "teleop.yaml"]
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"]
    )

    follower_robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " variant:=follower", " use_ros2_control:=false"]),
        value_type=str,
    )

    software_leader = Node(
        package="so101_bringup",
        executable="software_leader.py",
        namespace="leader",
        output="screen",
        arguments=[],
        condition=IfCondition(use_gui),
    )

    software_leader_headless = Node(
        package="so101_bringup",
        executable="software_leader.py",
        namespace="leader",
        output="screen",
        arguments=["--no-gui"],
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    follower_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=follower_ns,
        parameters=[
            {
                "robot_description": follower_robot_description,
                "frame_prefix": follower_frame_prefix,
            }
        ],
        output="screen",
        condition=IfCondition(launch_follower),
    )

    follower_bridge = Node(
        package="so101_hx35hm_bridge",
        executable="hx35hm_bridge",
        namespace=follower_ns,
        name="hx35hm_bridge",
        output="screen",
        parameters=[
            follower_params_file,
            {
                "device": follower_device,
            },
        ],
        condition=IfCondition(launch_follower),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_teleop"), "launch", "teleop.launch.py"])
        ),
        launch_arguments={
            "leader_namespace": "leader",
            "follower_namespace": follower_ns,
            "arm_controller": "forward_controller",
            "params_file": teleop_params_file,
        }.items(),
    )

    teleop_start = TimerAction(
        period=teleop_delay_s,
        actions=[teleop_launch],
        condition=IfCondition(use_teleop),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("follower_device", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("follower_params_file", default_value=default_follower_params),
            DeclareLaunchArgument("teleop_params_file", default_value=default_teleop_params),
            DeclareLaunchArgument("teleop_delay_s", default_value="2.0"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("launch_follower", default_value="true"),
            software_leader,
            software_leader_headless,
            follower_rsp,
            follower_bridge,
            teleop_start,
        ]
    )
