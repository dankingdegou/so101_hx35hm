import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    variant = LaunchConfiguration("variant")
    use_octomap = LaunchConfiguration("use_octomap")
    sensors_config = LaunchConfiguration("sensors_config")

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "variant": variant,
                "use_ros2_control": "false",
            },
        )
        .robot_description_semantic(file_path="config/so101_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"]) # loads ompl_planning.yaml
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False, # don't let MoveIt switch controllers
        )
        .to_moveit_configs()
    )

    def _make_move_group(context):
        use_octomap_val = str(use_octomap.perform(context)).lower() in ("1", "true", "yes", "on")
        use_sim_time_val = str(use_sim_time.perform(context)).lower() in ("1", "true", "yes", "on")

        params = [moveit_config.to_dict(), {"use_sim_time": use_sim_time_val}]
        if use_octomap_val:
            # Load sensors_3d.yaml (octomap_frame/resolution + sensors list).
            # This must be a parameter file so ROS can handle the list-of-dicts type.
            sensors_path = sensors_config.perform(context)
            params.insert(1, sensors_path)

        # Run in root namespace (MoveIt + namespaces is buggy, all reference
        # projects do this). Remap joint_states so move_group finds the topic
        # published inside the controller namespace.
        node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=params,
            remappings=[("joint_states", ["/", namespace, "/joint_states"])],
        )
        return [node]

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("variant", default_value="follower"),
            DeclareLaunchArgument("use_octomap", default_value="false"),
            DeclareLaunchArgument(
                "sensors_config",
                default_value=os.path.join(
                    get_package_share_directory("so101_moveit_config"),
                    "config",
                    "octomap_pointcloud.yaml",
                ),
            ),
            OpaqueFunction(function=_make_move_group),
        ]
    )
