from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch arguments ---
    hardware_type = LaunchConfiguration("hardware_type")  # real|mock|mujoco
    # When using HX-35HM on the follower, we still want controllers (forward_controller)
    # but we do NOT want the original hardware driver to touch a serial device.
    follower_hardware_type = LaunchConfiguration("follower_hardware_type")
    leader_ns = LaunchConfiguration("leader_namespace")
    follower_ns = LaunchConfiguration("follower_namespace")
    leader_frame_prefix = LaunchConfiguration("leader_frame_prefix")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")

    leader_usb = LaunchConfiguration("leader_usb_port")
    follower_usb = LaunchConfiguration("follower_usb_port")

    leader_joint_cfg = LaunchConfiguration("leader_joint_config_file")
    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")

    leader_ctrl_cfg = LaunchConfiguration("leader_controller_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")

    leader_rviz = LaunchConfiguration("leader_rviz")
    follower_rviz = LaunchConfiguration("follower_rviz")

    arm_controller = LaunchConfiguration("arm_controller")  # trajectory_controller|forward_controller

    teleop_params_file = LaunchConfiguration("teleop_params_file")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")

    use_cameras = LaunchConfiguration("use_cameras")
    cameras_config_file = LaunchConfiguration("cameras_config_file")
    use_camera_tf = LaunchConfiguration("use_camera_tf")

    use_teleop_rviz = LaunchConfiguration("use_teleop_rviz")

    # 使用 HX-35HM + STM32 控制板时的开关
    use_hx35hm = LaunchConfiguration("use_hx35hm")

    use_rerun = LaunchConfiguration("use_rerun")
    rerun_env_dir = LaunchConfiguration("rerun_env_dir")
    rerun_delay_s = LaunchConfiguration("rerun_delay_s")

    # --- Include leader bringup ---
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "leader.launch.py"])
        ),
        launch_arguments={
            "namespace": leader_ns,
            "hardware_type": hardware_type,
            "usb_port": leader_usb,
            "frame_prefix": leader_frame_prefix,
            "joint_config_file": leader_joint_cfg,
            "controller_config_file": leader_ctrl_cfg,
            "use_rviz": leader_rviz,
        }.items(),
    )

    # --- Include follower bringup ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "follower.launch.py"])
        ),
        launch_arguments={
            "namespace": follower_ns,
            "hardware_type": follower_hardware_type,
            "usb_port": follower_usb,
            "frame_prefix": follower_frame_prefix,
            "joint_config_file": follower_joint_cfg,
            "controller_config_file": follower_ctrl_cfg,
            "use_rviz": follower_rviz,
            "arm_controller": arm_controller,
            # When HX-35HM bridge publishes /follower/joint_states, avoid two publishers.
            "spawn_joint_state_broadcaster": PythonExpression(
                ["'false' if '", use_hx35hm, "' == 'true' else 'true'"]
            ),
        }.items(),
    )

    # --- Include teleop launch ---
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_teleop"), "launch", "teleop.launch.py"])
        ),
        launch_arguments={
            "leader_namespace": leader_ns,
            "follower_namespace": follower_ns,
            "arm_controller": arm_controller,
            "params_file": teleop_params_file,
        }.items(),
    )

    teleop_start = TimerAction(
        period=teleop_delay_s,
        actions=[teleop_include],
    )

    # --- Include cameras launch ---
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "cameras.launch.py"])
        ),
        condition=IfCondition(use_cameras),
        launch_arguments={
            "cameras_config": cameras_config_file,
        }.items(),
    )

    camera_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "camera_tf.launch.py"])
        ),
        condition=IfCondition(use_camera_tf),
        launch_arguments={}.items(),
    )

    # --- Include layout launch tf
    layout_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "layout_tf.launch.py"])
        ),
    )

    # --- Rviz Node

    teleop_rviz = PathJoinSubstitution([FindPackageShare("so101_bringup"), "rviz", "teleop.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="teleop_rviz",
        arguments=["-d", teleop_rviz],
        condition=IfCondition(use_teleop_rviz),
        output="screen",
    )

    # --- Launch Rerun

    rerun_bridge_proc = ExecuteProcess(
        cmd=[
            "pixi",
            "run",
            "bridge",
            "--",
            # "--wrist", rerun_wrist,
            # "--overhead", rerun_overhead,
            # "--joint-states", rerun_joint_states,
            # "--forward-commands", rerun_forward_cmds,
            # "--joint-trajectory", rerun_joint_traj,
        ],
        cwd=rerun_env_dir,
        additional_env={"PYTHONUNBUFFERED": "1"},
        condition=IfCondition(use_rerun),
        output="screen",
    )

    rerun_start = TimerAction(
        period=rerun_delay_s,
        actions=[rerun_bridge_proc],
    )

    # --- Defaults for files ---
    default_leader_joint_cfg = ""  # Optional; example default:
    # PathJoinSubstitution([FindPackageShare("so101_bringup"), "config", "hardware", "leader_joints.yaml"])
    default_follower_joint_cfg = ""  # Optional; example default:
    # PathJoinSubstitution([FindPackageShare("so101_bringup"), "config", "hardware", "follower_joints.yaml"])
    default_leader_ctrl_cfg = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "ros2_control",
            "leader_controllers.yaml",
        ]
    )
    default_follower_ctrl_cfg = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "ros2_control",
            "follower_controllers.yaml",
        ]
    )
    default_teleop_params = PathJoinSubstitution([FindPackageShare("so101_teleop"), "config", "teleop.yaml"])
    default_cameras_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "cameras", "so101_cameras.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument(
                "follower_hardware_type",
                # Safe default: when use_hx35hm is true, force follower to mock so we don't
                # accidentally also command Feetech hardware (or fail to connect).
                default_value=PythonExpression(
                    [
                        "'mock' if '",
                        use_hx35hm,
                        "' == 'true' else '",
                        hardware_type,
                        "'",
                    ]
                ),
                description="Follower hardware type. Defaults to 'mock' when use_hx35hm:=true, else mirrors hardware_type.",
            ),
            DeclareLaunchArgument("leader_namespace", default_value="leader"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("leader_frame_prefix", default_value="leader/"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("leader_usb_port", default_value="/dev/so101_leader"),
            DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("leader_joint_config_file", default_value=default_leader_joint_cfg),
            DeclareLaunchArgument("follower_joint_config_file", default_value=default_follower_joint_cfg),
            DeclareLaunchArgument("leader_controller_config_file", default_value=default_leader_ctrl_cfg),
            DeclareLaunchArgument(
                "follower_controller_config_file",
                default_value=default_follower_ctrl_cfg,
            ),
            DeclareLaunchArgument("leader_rviz", default_value="false"),
            DeclareLaunchArgument("follower_rviz", default_value="false"),
            DeclareLaunchArgument("arm_controller", default_value="forward_controller"),
            DeclareLaunchArgument("teleop_params_file", default_value=default_teleop_params),
            DeclareLaunchArgument("teleop_delay_s", default_value="2.0"),
            DeclareLaunchArgument("use_cameras", default_value="true"),
            DeclareLaunchArgument("cameras_config_file", default_value=default_cameras_cfg),
            DeclareLaunchArgument("use_camera_tf", default_value="true"),
            DeclareLaunchArgument("use_teleop_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_hx35hm",
                default_value="false",
                description="Whether to use HX-35HM bridge + ros_robot_controller for follower arm.",
            ),
            DeclareLaunchArgument("use_rerun", default_value="false"),
            DeclareLaunchArgument(
                "rerun_env_dir",
                # Best: set env var once, no need to pass each run:
                # export SO101_RERUN_ENV_DIR=/abs/path/to/tools/rerun_env
                default_value=EnvironmentVariable("SO101_RERUN_ENV_DIR", default_value=""),
            ),
            DeclareLaunchArgument("rerun_delay_s", default_value=teleop_delay_s),
            leader_launch,
            follower_launch,
            layout_tf_launch,
            cameras_launch,
            camera_tf_launch,
            rviz_node,
            rerun_start,
            teleop_start,
            # HX-35HM bus servo bridge (directly uses Board SDK; do not
            # launch ros_robot_controller here to avoid double-opening serial).
            Node(
                package="so101_hx35hm_bridge",
                executable="hx35hm_bridge",
                name="hx35hm_bridge",
                output="screen",
                parameters=[
                    {
                        "device": "/dev/ros_robot_controller",
                        "command_topic": "/follower/forward_controller/commands",
                        # Publish to the follower namespace so MoveIt/recording/inference
                        # (which default to /follower/joint_states) see the hardware state.
                        "publish_joint_states_topic": "/follower/joint_states",
                        # Teleop doesn't use MoveIt, so keep the action server off here to avoid
                        # having it appear under the root namespace.
                        "enable_follow_joint_trajectory": False,
                    }
                ],
                condition=IfCondition(use_hx35hm),
            ),
        ]
    )
