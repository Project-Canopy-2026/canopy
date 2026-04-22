import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file, generate_robot_api_params


def launch_setup(context, *args, **kwargs):

    sim_mode  = LaunchConfiguration("sim_mode")
    ee_link   = LaunchConfiguration("ee_link")
    sim_pot_x = LaunchConfiguration("sim_pot_x")
    sim_pot_y = LaunchConfiguration("sim_pot_y")
    sim_pot_z = LaunchConfiguration("sim_pot_z")

    is_sim = sim_mode.perform(context) in ("true", "True", "1")

    ee_link_val = ee_link.perform(context)
    if not ee_link_val:
        ee_link_val = "link_eef" if is_sim else "tool_tcp"

    config = os.path.join(
        get_package_share_directory("manipulation_pkg"),
        "config",
        "robot_params.yaml",
    )

    ros2_control_plugin = (
        "uf_robot_hardware/UFRobotFakeSystemHardware"
        if is_sim else
        "uf_robot_hardware/UFRobotSystemHardware"
    )

    controllers_name = "fake_controllers" if is_sim else "controllers"

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory("xarm_controller"),
            "config",
            "xarm7_controllers.yaml",
        ),
        prefix="",
        add_gripper=False,
        ros_namespace="",
        robot_type="xarm",
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=7,
        robot_type="xarm",
        hw_ns="xarm",
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
    ).to_moveit_configs()

    moveit_config_dict = moveit_config.to_dict()

    # Add MTC capability
    cap = "move_group/ExecuteTaskSolutionCapability"
    if cap not in moveit_config_dict.get("capabilities", ""):
        moveit_config_dict["capabilities"] = (
            (moveit_config_dict.get("capabilities", "") + " " + cap).strip()
        )

    # ─────────────────────────────────────────────────────────────
    # CORE SYSTEM NODES
    # ─────────────────────────────────────────────────────────────

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch",
                "_robot_description.launch.py",
            ])
        ),
        launch_arguments={
            "robot_description": yaml.dump(moveit_config.robot_description),
        }.items(),
    )

    robot_params_file = generate_robot_api_params(
        os.path.join(get_package_share_directory("xarm_api"), "config", "xarm_params.yaml"),
        os.path.join(get_package_share_directory("xarm_api"), "config", "xarm_user_params.yaml"),
        "",
        node_name="ufactory_driver",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_control_params,
            robot_params_file,
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("xarm_moveit_config"),
                "rviz",
                "moveit.rviz",
            ])
        ],
        parameters=[moveit_config_dict],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "link_base"],
    )

    gripper_node = Node(
        package="gripper_pkg",
        executable="gripper_node",
        parameters=[config, {"sim_mode": sim_mode}],
    )

    # ─────────────────────────────────────────────────────────────
    # SAFE CONTROLLER SPAWNERS (FIXED)
    # ─────────────────────────────────────────────────────────────

    def make_spawner(name, delay):
        return TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "controller_manager", "spawner",
                        name,
                        "--controller-manager",
                        "/controller_manager",
                    ],
                    output="screen",
                )
            ],
        )

    joint_state_broadcaster = make_spawner("joint_state_broadcaster", 5.0)
    xarm7_controller = make_spawner("xarm7_traj_controller", 6.0)

    # ─────────────────────────────────────────────────────────────
    # MTC NODE (already safe)
    # ─────────────────────────────────────────────────────────────

    mtc_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="manipulation_mtc",
                executable="pick_and_place_node",
                name="pick_and_place",
                output="screen",
                parameters=[
                    moveit_config_dict,
                    {
                        "sim_mode": sim_mode,
                        "ee_link": ee_link_val,
                        "sim_pot_x": sim_pot_x,
                        "sim_pot_y": sim_pot_y,
                        "sim_pot_z": sim_pot_z,
                    },
                ],
            )
        ],
    )

    return [
        robot_description,
        static_tf,
        ros2_control_node,
        move_group_node,
        rviz_node,
        gripper_node,
        joint_state_broadcaster,
        xarm7_controller,
        mtc_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_mode", default_value="false"),
        DeclareLaunchArgument("ee_link", default_value=""),
        DeclareLaunchArgument("sim_pot_x", default_value="0.0"),
        DeclareLaunchArgument("sim_pot_y", default_value="-0.350"),
        DeclareLaunchArgument("sim_pot_z", default_value="0.160"),
        OpaqueFunction(function=launch_setup),
    ])