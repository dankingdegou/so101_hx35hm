from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_namespace")
    follower_ns = LaunchConfiguration("follower_namespace")
    leader_device = LaunchConfiguration("leader_device")
    follower_device = LaunchConfiguration("follower_device")
    leader_frame_prefix = LaunchConfiguration("leader_frame_prefix")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    leader_params_file = LaunchConfiguration("leader_params_file")
    follower_params_file = LaunchConfiguration("follower_params_file")
    teleop_params_file = LaunchConfiguration("teleop_params_file")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")
    leader_rviz = LaunchConfiguration("leader_rviz")
    follower_rviz = LaunchConfiguration("follower_rviz")
    use_teleop_rviz = LaunchConfiguration("use_teleop_rviz")
    launch_leader = LaunchConfiguration("launch_leader")
    launch_follower = LaunchConfiguration("launch_follower")
    launch_follower_bridge = LaunchConfiguration("launch_follower_bridge")
    launch_teleop = LaunchConfiguration("launch_teleop")

    default_leader_params = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "hx35hm_leader_bridge_params.yaml",
        ]
    )
    default_teleop_params = PathJoinSubstitution(
        [FindPackageShare("so101_teleop"), "config", "teleop.yaml"]
    )
    default_follower_params = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "hx35hm_follower_bridge_params.yaml",
        ]
    )

    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "leader_hx35hm.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": leader_ns,
            "device": leader_device,
            "params_file": leader_params_file,
            "frame_prefix": leader_frame_prefix,
            "use_rviz": leader_rviz,
        }.items(),
        condition=IfCondition(launch_leader),
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"]
    )

    follower_robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " variant:=follower", " use_ros2_control:=false"]),
        value_type=str,
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
        condition=IfCondition(launch_follower_bridge),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_teleop"), "launch", "teleop.launch.py"])
        ),
        launch_arguments={
            "leader_namespace": leader_ns,
            "follower_namespace": follower_ns,
            "arm_controller": "forward_controller",
            "params_file": teleop_params_file,
        }.items(),
    )

    teleop_start = TimerAction(
        period=teleop_delay_s,
        actions=[teleop_launch],
        condition=IfCondition(launch_teleop),
    )

    teleop_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="teleop_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "rviz", "teleop.rviz"]
            ),
        ],
        output="screen",
        condition=IfCondition(use_teleop_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("leader_namespace", default_value="leader"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("leader_device", default_value="/dev/so101_leader"),
            DeclareLaunchArgument("follower_device", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("leader_frame_prefix", default_value="leader/"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("leader_params_file", default_value=default_leader_params),
            DeclareLaunchArgument("follower_params_file", default_value=default_follower_params),
            DeclareLaunchArgument("teleop_params_file", default_value=default_teleop_params),
            DeclareLaunchArgument("teleop_delay_s", default_value="2.0"),
            DeclareLaunchArgument("leader_rviz", default_value="false"),
            DeclareLaunchArgument("follower_rviz", default_value="false"),
            DeclareLaunchArgument("use_teleop_rviz", default_value="true"),
            DeclareLaunchArgument("launch_leader", default_value="true"),
            DeclareLaunchArgument("launch_follower", default_value="true"),
            DeclareLaunchArgument("launch_follower_bridge", default_value="true"),
            DeclareLaunchArgument("launch_teleop", default_value="true"),
            leader_launch,
            follower_rsp,
            follower_bridge,
            teleop_rviz,
            teleop_start,
        ]
    )
