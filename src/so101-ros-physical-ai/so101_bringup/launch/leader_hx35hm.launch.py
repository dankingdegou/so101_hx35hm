from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    device = LaunchConfiguration("device")
    params_file = LaunchConfiguration("params_file")
    frame_prefix = LaunchConfiguration("frame_prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " variant:=leader", " use_ros2_control:=false"]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description, "frame_prefix": frame_prefix}],
        output="screen",
    )

    leader_bridge = Node(
        package="so101_hx35hm_bridge",
        executable="hx35hm_bridge",
        namespace=namespace,
        name="hx35hm_bridge",
        output="screen",
        parameters=[
            params_file,
            {
                "device": device,
                "command_topic": "forward_controller/commands",
                "enable_command_subscription": False,
                "publish_joint_states_topic": "joint_states",
                "enable_follow_joint_trajectory": False,
                "enable_gripper_action": False,
                "enable_position_readback": True,
                "move_duration": 0.8,
                "stream_command_duration": 0.05,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="leader"),
            DeclareLaunchArgument("device", default_value="/dev/so101_leader"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("so101_bringup"), "config", "hx35hm_leader_bridge_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="leader/",
                description="TF frame prefix for robot_state_publisher.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("so101_bringup"), "rviz", "leader.rviz"]
                ),
            ),
            rsp,
            leader_bridge,
            rviz_node,
        ]
    )
