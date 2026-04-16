from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    device = LaunchConfiguration("device")
    command_topic = LaunchConfiguration("command_topic")
    state_topic = LaunchConfiguration("state_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value="/dev/ros_robot_controller",
                description="Serial device for STM32 bus-servo controller.",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/follower/forward_controller/commands",
                description="ForwardCommandController command topic for follower arm.",
            ),
            DeclareLaunchArgument(
                "state_topic",
                default_value="/joint_states",
                description="JointState topic to publish hardware joint states.",
            ),
            Node(
                package="so101_hx35hm_bridge",
                executable="hx35hm_bridge",
                name="hx35hm_bridge",
                output="screen",
                parameters=[
                    {
                        "device": device,
                        "command_topic": command_topic,
                        "publish_joint_states_topic": state_topic,
                    }
                ],
            ),
        ]
    )

