import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    joint_config_file = LaunchConfiguration("joint_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_octomap = LaunchConfiguration("use_octomap")
    sensors_config = LaunchConfiguration("sensors_config")
    use_camera_tf = LaunchConfiguration("use_camera_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    use_vision_debug_rviz = LaunchConfiguration("use_vision_debug_rviz")
    use_joint_gui = LaunchConfiguration("use_joint_gui")
    use_aruco_detector = LaunchConfiguration("use_aruco_detector")
    use_red_detector = LaunchConfiguration("use_red_detector")
    use_cameras = LaunchConfiguration("use_cameras")
    cameras_config = LaunchConfiguration("cameras_config")
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

    # 1) Robot description + state publisher（不使用 ros2_control）
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

    # 2) HX-35HM bridge 节点（运行在 follower 命名空间）
    hx_bridge = Node(
        package="so101_hx35hm_bridge",
        executable="hx35hm_bridge",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "device": "/dev/ros_robot_controller",
                "publish_joint_states_topic": "joint_states",
                # Safety-first slow mode for manual GUI/joint commands.
                "move_duration": 0.8,
                # The Cartesian motion node streams /forward_controller commands at 50 Hz.
                # Keep this short so /go_to_pose tracks targets instead of lagging.
                "stream_command_duration": 0.05,
                "enable_follow_joint_trajectory": True,
                "enable_gripper_action": True,
                "enable_position_readback": True,
            }
        ],
    )

    # 3) Move group（MoveIt）
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

    # 3.1) Cartesian IK / trajectory service for task-level grasping.
    # so101_grasping defaults to /go_to_pose; keep this service available in the
    # standard HX35HM + MoveIt bringup so task nodes do not silently fall back to
    # a different execution path.
    cartesian_motion = Node(
        package="so101_kinematics",
        executable="cartesian_motion_node",
        output="screen",
        parameters=[
            {
                "joints_topic": "/follower/joint_states",
                "cmd_topic": "/follower/forward_controller/commands",
                "base_frame": "base_link",
                "robot_description_package": "so101_description",
                "robot_description_xacro": "urdf/so101_arm.urdf.xacro",
                "robot_variant": "follower",
                # HX-35HM bus servos have measurable compliance/backlash. Keep the
                # service-level check realistic, then let task nodes decide which
                # stages need tighter semantic checks.
                "goal_wait_timeout_s": 1.5,
                "goal_position_tolerance_m": 0.035,
                "goal_orientation_tolerance_rad": 0.25,
            }
        ],
    )

    # 3.5) Camera TF for MoveIt (un-prefixed frames)
    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_bringup"),
                "launch",
                "camera_tf_moveit.launch.py",
            )
        ),
        condition=IfCondition(use_camera_tf),
        launch_arguments={
            "camera_x": camera_x,
            "camera_y": camera_y,
            "camera_z": camera_z,
            "camera_roll": camera_roll,
            "camera_pitch": camera_pitch,
            "camera_yaw": camera_yaw,
            "depth_to_rgb_x": depth_to_rgb_x,
            "depth_to_rgb_y": depth_to_rgb_y,
            "depth_to_rgb_z": depth_to_rgb_z,
            "depth_to_rgb_roll": depth_to_rgb_roll,
            "depth_to_rgb_pitch": depth_to_rgb_pitch,
            "depth_to_rgb_yaw": depth_to_rgb_yaw,
            "use_depth_to_rgb_tuner": use_depth_to_rgb_tuner,
        }.items(),
    )

    # 3.6) Cameras (optional)
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

    # 3.7) ArUco detector (optional, custom node with configurable dictionary)
    aruco_detector = Node(
        package="so101_hx35hm_bridge",
        executable="aruco_detector",
        output="screen",
        condition=IfCondition(use_aruco_detector),
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "image_topic": "/static_camera/image_raw",
                "camera_info_topic": "/static_camera/camera_info",
                "target_frame": "base_link",
                "marker_id": -1,
                "marker_size_m": 0.03,
                "dictionary": "DICT_ARUCO_ORIGINAL",
                "auto_dictionary": True,
                "max_message_age_s": 0.25,
                "min_marker_distance_m": 0.08,
                "max_marker_distance_m": 1.20,
                "smoothing_window": 5,
                "min_stable_samples": 3,
                "max_position_jump_m": 0.08,
                "output_ns": "/vision/aruco",
            }
        ],
    )

    red_detector = Node(
        package="so101_hx35hm_bridge",
        executable="red_circle_detector",
        output="screen",
        condition=IfCondition(use_red_detector),
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "image_topic": "/static_camera/image_raw",
                "depth_topic": "/static_camera/depth/image_raw",
                "camera_info_topic": "/static_camera/depth/camera_info",
                "rgb_camera_info_topic": "/static_camera/camera_info",
                "target_frame": "base_link",
                "output_ns": "/vision/red_block",
                "min_area_px": 200.0,
                "min_circularity": 0.60,
                "min_depth_m": 0.10,
                "max_depth_m": 1.50,
                "max_z_m": 0.08,
                # Overhead RGB comes from a separate UVC stream while depth comes from OpenNI2.
                # Use projective registration (depth pixel -> 3D -> RGB reprojection) instead of
                # the old pure scale heuristic so x/y can follow image position more faithfully.
                "pixel_mapping_mode": "projective",
                # Optional depth->rgb extrinsic refinement. Identity is a safe starting point;
                # if you later calibrate RGB-depth extrinsics you can drop the measured values here.
                "depth_to_rgb_translation_m": [0.0, 0.0, 0.0],
                "depth_to_rgb_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "registration_max_residual_px": 14.0,
                # This RGBD chain is not tightly synchronized; allow older frames for static debugging.
                "max_message_age_s": 2.00,
                # Disable RGB-depth skew rejection for static tabletop debugging on this camera pair.
                "max_rgb_depth_skew_s": 0.0,
                # Registration still begins from a local seed; keep a moderate window for nearby candidates.
                "depth_search_radius_px": 28,
                # Keep tabletop fallback as a safety net, but with projective registration the detector
                # should stay on real depth more often than before.
                "enable_plane_fallback": True,
                "fallback_plane_z_m": 0.0,
                "smoothing_window": 5,
                "min_stable_samples": 3,
                "max_position_jump_m": 0.10,
                "h1_lower": [0, 120, 70],
                "h1_upper": [8, 255, 255],
                "h2_lower": [172, 120, 70],
                "h2_upper": [180, 255, 255],
            }
        ],
    )

    table_estimator = Node(
        package="so101_hx35hm_bridge",
        executable="table_plane_estimator",
        output="screen",
        condition=IfCondition(use_red_detector),
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "depth_topic": "/static_camera/depth/image_raw",
                "camera_info_topic": "/static_camera/depth/camera_info",
                "target_frame": "base_link",
                "output_topic": "/vision/table/top_z",
                "output_pose_topic": "/vision/table/pose_base",
                "status_topic": "/vision/table/status",
                "max_message_age_s": 2.0,
                "roi_u_min": 0.30,
                "roi_u_max": 0.70,
                "roi_v_min": 0.55,
                "roi_v_max": 0.92,
                "sample_step_px": 6,
                "min_depth_m": 0.10,
                "max_depth_m": 1.50,
                "min_points": 80,
                "z_trim_margin_m": 0.015,
                "smoothing_window": 5,
                "publish_period_s": 0.25,
            }
        ],
    )

    # 4) MoveIt RViz
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

    vision_debug_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "rviz", "vision_overhead_debug.rviz"]),
        ],
        output="screen",
        condition=IfCondition(use_vision_debug_rviz),
    )

    # 5) Joint slider GUI (optional)
    joint_gui = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "scripts", "so101_joint_gui.py"]),
            "--command-topic",
            "/follower/forward_controller/commands",
            "--joint-state-topic",
            "/joint_states",
        ],
        output="screen",
        condition=IfCondition(use_joint_gui),
    )

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
            DeclareLaunchArgument("use_vision_debug_rviz", default_value="false"),
            DeclareLaunchArgument("use_joint_gui", default_value="true"),
            DeclareLaunchArgument("use_aruco_detector", default_value="true"),
            DeclareLaunchArgument("use_red_detector", default_value="false"),
            DeclareLaunchArgument("use_cameras", default_value="false"),
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
            DeclareLaunchArgument(
                "cameras_config",
                default_value=os.path.join(
                    get_package_share_directory("so101_bringup"),
                    "config",
                    "cameras",
                    "so101_cameras_astra_overhead_rgbd.yaml",
                ),
            ),
            rsp,
            hx_bridge,
            camera_tf,
            cameras_launch,
            aruco_detector,
            red_detector,
            table_estimator,
            cartesian_motion,
            move_group,
            moveit_rviz,
            vision_debug_rviz,
            joint_gui,
        ]
    )
