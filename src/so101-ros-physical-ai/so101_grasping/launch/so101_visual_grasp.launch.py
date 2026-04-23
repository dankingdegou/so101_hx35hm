from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_topic = LaunchConfiguration("pose_topic")
    execute = LaunchConfiguration("execute")
    ik_tip_link = LaunchConfiguration("ik_tip_link")
    wait_pose_timeout_s = LaunchConfiguration("wait_pose_timeout_s")
    fresh_pose_slack_s = LaunchConfiguration("fresh_pose_slack_s")
    pose_sample_window_s = LaunchConfiguration("pose_sample_window_s")
    pose_sample_count = LaunchConfiguration("pose_sample_count")
    min_pose_sample_count = LaunchConfiguration("min_pose_sample_count")
    max_pose_spread_m = LaunchConfiguration("max_pose_spread_m")
    post_rest_settle_s = LaunchConfiguration("post_rest_settle_s")
    grasp_x_offset_m = LaunchConfiguration("grasp_x_offset_m")
    grasp_y_offset_m = LaunchConfiguration("grasp_y_offset_m")
    grasp_z_offset_m = LaunchConfiguration("grasp_z_offset_m")
    pregrasp_offset_m = LaunchConfiguration("pregrasp_offset_m")
    hover_high_offset_m = LaunchConfiguration("hover_high_offset_m")
    pregrasp_x_offset_m = LaunchConfiguration("pregrasp_x_offset_m")
    pregrasp_y_offset_m = LaunchConfiguration("pregrasp_y_offset_m")
    pregrasp_z_extra_m = LaunchConfiguration("pregrasp_z_extra_m")
    use_two_stage_pregrasp = LaunchConfiguration("use_two_stage_pregrasp")
    continuous_approach_mode = LaunchConfiguration("continuous_approach_mode")
    retreat_to_hover_high = LaunchConfiguration("retreat_to_hover_high")
    enforce_table_z_limits = LaunchConfiguration("enforce_table_z_limits")
    tabletop_z_m = LaunchConfiguration("tabletop_z_m")
    use_tabletop_z_topic = LaunchConfiguration("use_tabletop_z_topic")
    tabletop_z_topic = LaunchConfiguration("tabletop_z_topic")
    wait_tabletop_z_timeout_s = LaunchConfiguration("wait_tabletop_z_timeout_s")
    fresh_tabletop_z_slack_s = LaunchConfiguration("fresh_tabletop_z_slack_s")
    min_grasp_clearance_m = LaunchConfiguration("min_grasp_clearance_m")
    min_pregrasp_clearance_m = LaunchConfiguration("min_pregrasp_clearance_m")
    add_table_collision = LaunchConfiguration("add_table_collision")
    table_center_x_m = LaunchConfiguration("table_center_x_m")
    table_center_y_m = LaunchConfiguration("table_center_y_m")
    table_size_x_m = LaunchConfiguration("table_size_x_m")
    table_size_y_m = LaunchConfiguration("table_size_y_m")
    table_thickness_m = LaunchConfiguration("table_thickness_m")
    planning_time_s = LaunchConfiguration("planning_time_s")
    vel_scaling = LaunchConfiguration("vel_scaling")
    acc_scaling = LaunchConfiguration("acc_scaling")
    use_ik_pose_service = LaunchConfiguration("use_ik_pose_service")
    ik_pose_service_name = LaunchConfiguration("ik_pose_service_name")
    ik_pose_strategy = LaunchConfiguration("ik_pose_strategy")
    ik_pregrasp_strategy = LaunchConfiguration("ik_pregrasp_strategy")
    ik_grasp_strategy = LaunchConfiguration("ik_grasp_strategy")
    ik_retreat_strategy = LaunchConfiguration("ik_retreat_strategy")
    ik_pregrasp_duration_s = LaunchConfiguration("ik_pregrasp_duration_s")
    ik_grasp_duration_s = LaunchConfiguration("ik_grasp_duration_s")
    ik_retreat_duration_s = LaunchConfiguration("ik_retreat_duration_s")
    ik_pose_service_timeout_s = LaunchConfiguration("ik_pose_service_timeout_s")
    ik_pregrasp_settle_s = LaunchConfiguration("ik_pregrasp_settle_s")
    ik_grasp_settle_s = LaunchConfiguration("ik_grasp_settle_s")
    ik_retreat_settle_s = LaunchConfiguration("ik_retreat_settle_s")
    position_only = LaunchConfiguration("position_only")
    go_to_rest_before_grasp = LaunchConfiguration("go_to_rest_before_grasp")
    staging_named_pose = LaunchConfiguration("staging_named_pose")
    return_to_named_pose_after_grasp = LaunchConfiguration("return_to_named_pose_after_grasp")
    post_grasp_named_pose = LaunchConfiguration("post_grasp_named_pose")
    post_grasp_return_vel_scaling = LaunchConfiguration("post_grasp_return_vel_scaling")
    post_grasp_return_acc_scaling = LaunchConfiguration("post_grasp_return_acc_scaling")
    post_grasp_use_ik_joints = LaunchConfiguration("post_grasp_use_ik_joints")
    ik_joints_service_name = LaunchConfiguration("ik_joints_service_name")
    ik_post_grasp_duration_s = LaunchConfiguration("ik_post_grasp_duration_s")
    post_grasp_lift_before_return = LaunchConfiguration("post_grasp_lift_before_return")
    post_grasp_return_lift_z_m = LaunchConfiguration("post_grasp_return_lift_z_m")
    post_grasp_return_lift_extra_m = LaunchConfiguration("post_grasp_return_lift_extra_m")
    ik_post_grasp_lift_strategy = LaunchConfiguration("ik_post_grasp_lift_strategy")
    ik_post_grasp_lift_duration_s = LaunchConfiguration("ik_post_grasp_lift_duration_s")
    ik_post_grasp_lift_settle_s = LaunchConfiguration("ik_post_grasp_lift_settle_s")
    post_grasp_return_via_rest_hover = LaunchConfiguration("post_grasp_return_via_rest_hover")
    post_grasp_rest_hover_x_m = LaunchConfiguration("post_grasp_rest_hover_x_m")
    post_grasp_rest_hover_y_m = LaunchConfiguration("post_grasp_rest_hover_y_m")
    post_grasp_rest_hover_z_m = LaunchConfiguration("post_grasp_rest_hover_z_m")
    ik_post_grasp_rest_hover_strategy = LaunchConfiguration("ik_post_grasp_rest_hover_strategy")
    ik_post_grasp_rest_hover_duration_s = LaunchConfiguration("ik_post_grasp_rest_hover_duration_s")
    ik_post_grasp_rest_hover_settle_s = LaunchConfiguration("ik_post_grasp_rest_hover_settle_s")
    use_marker_orientation = LaunchConfiguration("use_marker_orientation")
    use_rpy = LaunchConfiguration("use_rpy")
    qx = LaunchConfiguration("qx")
    qy = LaunchConfiguration("qy")
    qz = LaunchConfiguration("qz")
    qw = LaunchConfiguration("qw")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    grasp_retry_count = LaunchConfiguration("grasp_retry_count")
    grasp_retry_delay_s = LaunchConfiguration("grasp_retry_delay_s")
    post_grasp_return_retry_count = LaunchConfiguration("post_grasp_return_retry_count")
    post_grasp_return_retry_delay_s = LaunchConfiguration("post_grasp_return_retry_delay_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_topic", default_value="/vision/red_block/pose_base"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("ik_tip_link", default_value="gripper_frame_link"),
            DeclareLaunchArgument("wait_pose_timeout_s", default_value="10.0"),
            DeclareLaunchArgument("fresh_pose_slack_s", default_value="2.5"),
            DeclareLaunchArgument("pose_sample_window_s", default_value="0.35"),
            DeclareLaunchArgument("pose_sample_count", default_value="5"),
            DeclareLaunchArgument("min_pose_sample_count", default_value="3"),
            DeclareLaunchArgument("max_pose_spread_m", default_value="0.03"),
            DeclareLaunchArgument("post_rest_settle_s", default_value="0.4"),
            DeclareLaunchArgument("grasp_x_offset_m", default_value="0.0"),
            DeclareLaunchArgument("grasp_y_offset_m", default_value="0.0"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="-0.015"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("hover_high_offset_m", default_value="0.04"),
            DeclareLaunchArgument("pregrasp_x_offset_m", default_value="0.0"),
            DeclareLaunchArgument("pregrasp_y_offset_m", default_value="0.0"),
            DeclareLaunchArgument("pregrasp_z_extra_m", default_value="0.0"),
            DeclareLaunchArgument("use_two_stage_pregrasp", default_value="true"),
            DeclareLaunchArgument("continuous_approach_mode", default_value="true"),
            DeclareLaunchArgument("retreat_to_hover_high", default_value="true"),
            DeclareLaunchArgument("enforce_table_z_limits", default_value="true"),
            DeclareLaunchArgument("tabletop_z_m", default_value="0.0"),
            DeclareLaunchArgument("use_tabletop_z_topic", default_value="true"),
            DeclareLaunchArgument("tabletop_z_topic", default_value="/vision/table/top_z"),
            DeclareLaunchArgument("wait_tabletop_z_timeout_s", default_value="2.0"),
            DeclareLaunchArgument("fresh_tabletop_z_slack_s", default_value="1.0"),
            DeclareLaunchArgument("min_grasp_clearance_m", default_value="0.010"),
            DeclareLaunchArgument("min_pregrasp_clearance_m", default_value="0.060"),
            DeclareLaunchArgument("add_table_collision", default_value="true"),
            DeclareLaunchArgument("table_center_x_m", default_value="0.32"),
            DeclareLaunchArgument("table_center_y_m", default_value="0.0"),
            DeclareLaunchArgument("table_size_x_m", default_value="0.36"),
            DeclareLaunchArgument("table_size_y_m", default_value="0.45"),
            DeclareLaunchArgument("table_thickness_m", default_value="0.04"),
            DeclareLaunchArgument("planning_time_s", default_value="8.0"),
            DeclareLaunchArgument("vel_scaling", default_value="0.15"),
            DeclareLaunchArgument("acc_scaling", default_value="0.15"),
            DeclareLaunchArgument("use_ik_pose_service", default_value="true"),
            DeclareLaunchArgument("ik_pose_service_name", default_value="/go_to_pose"),
            DeclareLaunchArgument("ik_pose_strategy", default_value="joint_quintic"),
            DeclareLaunchArgument("ik_pregrasp_strategy", default_value="joint_quintic"),
            DeclareLaunchArgument("ik_grasp_strategy", default_value="cartesian"),
            DeclareLaunchArgument("ik_retreat_strategy", default_value="cartesian"),
            DeclareLaunchArgument("ik_pregrasp_duration_s", default_value="1.7"),
            DeclareLaunchArgument("ik_grasp_duration_s", default_value="1.1"),
            DeclareLaunchArgument("ik_retreat_duration_s", default_value="1.1"),
            DeclareLaunchArgument("ik_pose_service_timeout_s", default_value="8.0"),
            DeclareLaunchArgument("ik_pregrasp_settle_s", default_value="0.08"),
            DeclareLaunchArgument("ik_grasp_settle_s", default_value="0.04"),
            DeclareLaunchArgument("ik_retreat_settle_s", default_value="0.04"),
            DeclareLaunchArgument("position_only", default_value="true"),
            DeclareLaunchArgument("go_to_rest_before_grasp", default_value="false"),
            DeclareLaunchArgument("staging_named_pose", default_value="extended"),
            DeclareLaunchArgument("return_to_named_pose_after_grasp", default_value="false"),
            DeclareLaunchArgument("post_grasp_named_pose", default_value="rest"),
            DeclareLaunchArgument("post_grasp_return_vel_scaling", default_value="0.12"),
            DeclareLaunchArgument("post_grasp_return_acc_scaling", default_value="0.12"),
            DeclareLaunchArgument("open_gripper_after_return", default_value="true"),
            DeclareLaunchArgument("post_return_gripper_settle_s", default_value="0.15"),
            DeclareLaunchArgument("post_grasp_lift_before_return", default_value="false"),
            DeclareLaunchArgument("post_grasp_return_lift_z_m", default_value="0.14"),
            DeclareLaunchArgument("post_grasp_return_lift_extra_m", default_value="0.04"),
            DeclareLaunchArgument("ik_post_grasp_lift_strategy", default_value="cartesian"),
            DeclareLaunchArgument("ik_post_grasp_lift_duration_s", default_value="1.2"),
            DeclareLaunchArgument("ik_post_grasp_lift_settle_s", default_value="0.05"),
            DeclareLaunchArgument("post_grasp_return_via_rest_hover", default_value="false"),
            DeclareLaunchArgument("post_grasp_rest_hover_x_m", default_value="0.20"),
            DeclareLaunchArgument("post_grasp_rest_hover_y_m", default_value="0.0"),
            DeclareLaunchArgument("post_grasp_rest_hover_z_m", default_value="0.14"),
            DeclareLaunchArgument("ik_post_grasp_rest_hover_strategy", default_value="cartesian"),
            DeclareLaunchArgument("ik_post_grasp_rest_hover_duration_s", default_value="1.4"),
            DeclareLaunchArgument("ik_post_grasp_rest_hover_settle_s", default_value="0.05"),
            DeclareLaunchArgument("post_grasp_use_ik_joints", default_value="false"),
            DeclareLaunchArgument("ik_joints_service_name", default_value="/go_to_joints"),
            DeclareLaunchArgument("ik_post_grasp_duration_s", default_value="2.4"),
            DeclareLaunchArgument("use_marker_orientation", default_value="false"),
            DeclareLaunchArgument("use_rpy", default_value="false"),
            DeclareLaunchArgument("qx", default_value="0.0"),
            DeclareLaunchArgument("qy", default_value="1.0"),
            DeclareLaunchArgument("qz", default_value="0.0"),
            DeclareLaunchArgument("qw", default_value="0.0"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("grasp_retry_count", default_value="1"),
            DeclareLaunchArgument("grasp_retry_delay_s", default_value="0.5"),
            DeclareLaunchArgument("post_grasp_return_retry_count", default_value="3"),
            DeclareLaunchArgument("post_grasp_return_retry_delay_s", default_value="1.0"),
            Node(
                package="so101_grasping",
                executable="so101_visual_grasp",
                output="screen",
                remappings=[("joint_states", "/follower/joint_states")],
                parameters=[
                    {
                        "pose_topic": pose_topic,
                        "execute": execute,
                        "ik_tip_link": ik_tip_link,
                        "wait_pose_timeout_s": wait_pose_timeout_s,
                        "fresh_pose_slack_s": fresh_pose_slack_s,
                        "pose_sample_window_s": pose_sample_window_s,
                        "pose_sample_count": pose_sample_count,
                        "min_pose_sample_count": min_pose_sample_count,
                        "max_pose_spread_m": max_pose_spread_m,
                        "post_rest_settle_s": post_rest_settle_s,
                        "grasp_x_offset_m": grasp_x_offset_m,
                        "grasp_y_offset_m": grasp_y_offset_m,
                        "grasp_z_offset_m": grasp_z_offset_m,
                        "pregrasp_offset_m": pregrasp_offset_m,
                        "hover_high_offset_m": hover_high_offset_m,
                        "pregrasp_x_offset_m": pregrasp_x_offset_m,
                        "pregrasp_y_offset_m": pregrasp_y_offset_m,
                        "pregrasp_z_extra_m": pregrasp_z_extra_m,
                        "use_two_stage_pregrasp": use_two_stage_pregrasp,
                        "continuous_approach_mode": continuous_approach_mode,
                        "retreat_to_hover_high": retreat_to_hover_high,
                        "enforce_table_z_limits": enforce_table_z_limits,
                        "tabletop_z_m": tabletop_z_m,
                        "use_tabletop_z_topic": use_tabletop_z_topic,
                        "tabletop_z_topic": tabletop_z_topic,
                        "wait_tabletop_z_timeout_s": wait_tabletop_z_timeout_s,
                        "fresh_tabletop_z_slack_s": fresh_tabletop_z_slack_s,
                        "min_grasp_clearance_m": min_grasp_clearance_m,
                        "min_pregrasp_clearance_m": min_pregrasp_clearance_m,
                        "add_table_collision": add_table_collision,
                        "table_center_x_m": table_center_x_m,
                        "table_center_y_m": table_center_y_m,
                        "table_size_x_m": table_size_x_m,
                        "table_size_y_m": table_size_y_m,
                        "table_thickness_m": table_thickness_m,
                        "planning_time_s": planning_time_s,
                        "vel_scaling": vel_scaling,
                        "acc_scaling": acc_scaling,
                        "use_ik_pose_service": use_ik_pose_service,
                        "ik_pose_service_name": ik_pose_service_name,
                        "ik_pose_strategy": ik_pose_strategy,
                        "ik_pregrasp_strategy": ik_pregrasp_strategy,
                        "ik_grasp_strategy": ik_grasp_strategy,
                        "ik_retreat_strategy": ik_retreat_strategy,
                        "ik_pregrasp_duration_s": ik_pregrasp_duration_s,
                        "ik_grasp_duration_s": ik_grasp_duration_s,
                        "ik_retreat_duration_s": ik_retreat_duration_s,
                        "ik_pose_service_timeout_s": ik_pose_service_timeout_s,
                        "ik_pregrasp_settle_s": ik_pregrasp_settle_s,
                        "ik_grasp_settle_s": ik_grasp_settle_s,
                        "ik_retreat_settle_s": ik_retreat_settle_s,
                        "position_only": position_only,
                        "go_to_rest_before_grasp": go_to_rest_before_grasp,
                        "staging_named_pose": staging_named_pose,
                        "return_to_named_pose_after_grasp": return_to_named_pose_after_grasp,
                        "post_grasp_named_pose": post_grasp_named_pose,
                        "post_grasp_return_vel_scaling": post_grasp_return_vel_scaling,
                        "post_grasp_return_acc_scaling": post_grasp_return_acc_scaling,
                        "open_gripper_after_return": LaunchConfiguration("open_gripper_after_return"),
                        "post_return_gripper_settle_s": LaunchConfiguration("post_return_gripper_settle_s"),
                        "post_grasp_lift_before_return": post_grasp_lift_before_return,
                        "post_grasp_return_lift_z_m": post_grasp_return_lift_z_m,
                        "post_grasp_return_lift_extra_m": post_grasp_return_lift_extra_m,
                        "ik_post_grasp_lift_strategy": ik_post_grasp_lift_strategy,
                        "ik_post_grasp_lift_duration_s": ik_post_grasp_lift_duration_s,
                        "ik_post_grasp_lift_settle_s": ik_post_grasp_lift_settle_s,
                        "post_grasp_return_via_rest_hover": post_grasp_return_via_rest_hover,
                        "post_grasp_rest_hover_x_m": post_grasp_rest_hover_x_m,
                        "post_grasp_rest_hover_y_m": post_grasp_rest_hover_y_m,
                        "post_grasp_rest_hover_z_m": post_grasp_rest_hover_z_m,
                        "ik_post_grasp_rest_hover_strategy": ik_post_grasp_rest_hover_strategy,
                        "ik_post_grasp_rest_hover_duration_s": ik_post_grasp_rest_hover_duration_s,
                        "ik_post_grasp_rest_hover_settle_s": ik_post_grasp_rest_hover_settle_s,
                        "post_grasp_use_ik_joints": post_grasp_use_ik_joints,
                        "ik_joints_service_name": ik_joints_service_name,
                        "ik_post_grasp_duration_s": ik_post_grasp_duration_s,
                        "use_marker_orientation": use_marker_orientation,
                        "use_rpy": use_rpy,
                        "qx": qx,
                        "qy": qy,
                        "qz": qz,
                        "qw": qw,
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "grasp_retry_count": grasp_retry_count,
                        "grasp_retry_delay_s": grasp_retry_delay_s,
                        "post_grasp_return_retry_count": post_grasp_return_retry_count,
                        "post_grasp_return_retry_delay_s": post_grasp_return_retry_delay_s,
                    }
                ],
            ),
        ]
    )
