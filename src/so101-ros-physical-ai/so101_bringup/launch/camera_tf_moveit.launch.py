from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # This TF layout matches the un-prefixed URDF/SRDF used by MoveIt in this workspace.
    # Default overhead camera transform updated from the latest accepted
    # Hand-eye solve result can be loaded from a workspace-local aruco_handeye_result_v2.json.
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    depth_to_rgb_x = LaunchConfiguration("depth_to_rgb_x")
    depth_to_rgb_y = LaunchConfiguration("depth_to_rgb_y")
    depth_to_rgb_z = LaunchConfiguration("depth_to_rgb_z")
    depth_to_rgb_roll = LaunchConfiguration("depth_to_rgb_roll")
    depth_to_rgb_pitch = LaunchConfiguration("depth_to_rgb_pitch")
    depth_to_rgb_yaw = LaunchConfiguration("depth_to_rgb_yaw")
    use_depth_to_rgb_tuner = LaunchConfiguration("use_depth_to_rgb_tuner")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_x", default_value="0.519544"),
            DeclareLaunchArgument("camera_y", default_value="-0.013324"),
            DeclareLaunchArgument("camera_z", default_value="0.441101"),
            DeclareLaunchArgument("camera_roll", default_value="-2.8206266041480363"),
            DeclareLaunchArgument("camera_pitch", default_value="0.057491145560693215"),
            DeclareLaunchArgument("camera_yaw", default_value="1.6127540420128401"),
            DeclareLaunchArgument("depth_to_rgb_x", default_value="0.010000"),
            DeclareLaunchArgument("depth_to_rgb_y", default_value="-0.014000"),
            DeclareLaunchArgument("depth_to_rgb_z", default_value="0.0"),
            DeclareLaunchArgument("depth_to_rgb_roll", default_value="0.0"),
            DeclareLaunchArgument("depth_to_rgb_pitch", default_value="0.0"),
            DeclareLaunchArgument("depth_to_rgb_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_depth_to_rgb_tuner", default_value="false"),
            # Overhead camera pose in the world frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_overhead_cam_moveit",
                arguments=[
                    "--x", camera_x,
                    "--y", camera_y,
                    "--z", camera_z,
                    "--roll", camera_roll,
                    "--pitch", camera_pitch,
                    "--yaw", camera_yaw,
                    # Use base_link as parent to avoid needing an explicit world->base_link TF.
                    "--frame-id", "base_link",
                    "--child-frame-id", "cam_overhead",
                ],
            ),
            # Depth optical frame relative to the RGB overhead camera frame.
            # Keep it explicit even when initialized to identity so RGB-D registration
            # has a stable place to attach future calibration results.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_overhead_depth_to_rgb",
                condition=UnlessCondition(use_depth_to_rgb_tuner),
                arguments=[
                    "--x", depth_to_rgb_x,
                    "--y", depth_to_rgb_y,
                    "--z", depth_to_rgb_z,
                    "--roll", depth_to_rgb_roll,
                    "--pitch", depth_to_rgb_pitch,
                    "--yaw", depth_to_rgb_yaw,
                    "--frame-id", "cam_overhead",
                    "--child-frame-id", "cam_overhead_depth",
                ],
            ),
            ExecuteProcess(
                condition=IfCondition(use_depth_to_rgb_tuner),
                cmd=[
                    "python3",
                    PathJoinSubstitution(
                        [FindPackageShare("so101_bringup"), "scripts", "so101_depth_rgb_tuner.py"]
                    ),
                    "--parent-frame",
                    "cam_overhead",
                    "--child-frame",
                    "cam_overhead_depth",
                    "--x",
                    depth_to_rgb_x,
                    "--y",
                    depth_to_rgb_y,
                    "--z",
                    depth_to_rgb_z,
                    "--roll",
                    depth_to_rgb_roll,
                    "--pitch",
                    depth_to_rgb_pitch,
                    "--yaw",
                    depth_to_rgb_yaw,
                ],
                output="screen",
            ),
            # Wrist camera pose relative to the end-effector link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_wrist_cam_moveit",
                arguments=[
                    "--x", "0.00",
                    "--y", "0.0",
                    "--z", "-0.02",
                    "--roll", "-1.57",
                    "--pitch", "0.0",
                    "--yaw", "-1.57",
                    "--frame-id", "moving_jaw_so101_v1_link",
                    "--child-frame-id", "cam_wrist",
                ],
            ),
        ]
    )
