import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_octomap = LaunchConfiguration("use_octomap")
    sensors_config = LaunchConfiguration("sensors_config")
    use_camera_tf = LaunchConfiguration("use_camera_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    use_cameras = LaunchConfiguration("use_cameras")
    cameras_config = LaunchConfiguration("cameras_config")

    # 1) Robot description + state publisher
    xacro_file = PathJoinSubstitution([FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " variant:=follower", " use_ros2_control:=false"]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # 2) HX-35HM bridge node - START FIRST
    hx_bridge = Node(
        package="so101_hx35hm_bridge",
        executable="hx35hm_bridge",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "device": "/dev/ros_robot_controller",
                "publish_joint_states_topic": "joint_states",
                "enable_follow_joint_trajectory": True,
                "enable_gripper_action": True,
                "enable_position_readback": True,
            }
        ],
    )

    # 3) Move group - START AFTER BRIDGE IS READY
    # We use a delay to ensure bridge is fully initialized
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "variant": "follower",
            "use_sim_time": use_sim_time,
            "use_octomap": use_octomap,
            "sensors_config": sensors_config,
        }.items(),
    )

    # 4) Camera TF for MoveIt
    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_bringup"),
                "launch",
                "camera_tf_moveit.launch.py",
            )
        ),
        condition=IfCondition(use_camera_tf),
    )

    # 5) Cameras (optional)
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_bringup"),
                "launch",
                "cameras.launch.py",
            )
        ),
        condition=IfCondition(use_cameras),
        launch_arguments={
            "cameras_config": cameras_config,
        }.items(),
    )

    # 6) MoveIt RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "launch",
                "moveit_rviz.launch.py",
            )
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "variant": "follower",
        }.items(),
    )

    # Delay move_group start to ensure bridge is ready
    delayed_move_group = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hx_bridge,
            on_start=[
                # Wait 5 seconds after bridge starts before launching move_group
                Node(
                    package="so101_bringup",
                    executable="delayed_start",
                    arguments=["5", "move_group_start"],
                    on_exit=[move_group, moveit_rviz],
                ),
            ],
        )
    )

    # Alternative: Simple delay using sleep in shell
    # We'll use a shell-based approach instead
    from launch.actions import ExecuteProcess
    import subprocess
    
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("joint_config_file", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_octomap", default_value="false"),
            DeclareLaunchArgument(
                "sensors_config",
                default_value=os.path.join(
                    get_package_share_directory("so101_moveit_config"),
                    "config",
                    "octomap_pointcloud.yaml",
                ),
            ),
            DeclareLaunchArgument("use_camera_tf", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_cameras", default_value="false"),
            DeclareLaunchArgument(
                "cameras_config",
                default_value=os.path.join(
                    get_package_share_directory("so101_bringup"),
                    "config",
                    "cameras",
                    "so101_cameras_astra_overhead_rgbd_lowbw.yaml",
                ),
            ),
            rsp,
            hx_bridge,
            camera_tf,
            cameras_launch,
            # move_group and rviz will be started manually after bridge is ready
            # OR use the follower_hx35hm_moveit_2step.sh script
        ]
    )