"""
arm_bringup.launch.py

Full xarm7 bringup with MoveIt2 + MTC pick-and-place node.

Replaces the old planner_node / deterministic_planner_node with the
MTC-based pick_and_place_node.  move_group is started with the
move_group/ExecuteTaskSolutionCapability plugin so that MTC's
task.execute() can reach the execute_task_solution action server.

Usage (simulation / no hardware):
  ros2 launch manipulation_mtc arm_bringup.launch.py sim_mode:=true

Usage (real robot):
  ros2 launch manipulation_mtc arm_bringup.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    sim_mode  = LaunchConfiguration("sim_mode")
    robot_ip  = LaunchConfiguration("robot_ip")
    ee_link   = LaunchConfiguration("ee_link")
    sim_pot_x = LaunchConfiguration("sim_pot_x")
    sim_pot_y = LaunchConfiguration("sim_pot_y")
    sim_pot_z = LaunchConfiguration("sim_pot_z")

    is_sim = sim_mode.perform(context) in ("true", "True", "1")

    # Sim URDF only has 'link_eef'; real robot adds the custom 'tool_tcp' TCP.
    # If the user explicitly passed ee_link, honour it; otherwise pick the right default.
    ee_link_val = ee_link.perform(context)
    if not ee_link_val:
        ee_link_val = "link_eef" if is_sim else "tool_tcp"

    config = os.path.join(
        get_package_share_directory("manipulation_pkg"),
        "config",
        "robot_params.yaml",
    )

    # ── Build MoveIt config ───────────────────────────────────────────────
    ros2_control_plugin = (
        "uf_robot_hardware/UFRobotFakeSystemHardware"
        if is_sim
        else "uf_robot_hardware/UFRobotSystemHardware"
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
        add_bio_gripper=False,
        ros_namespace="",
        robot_type="xarm",
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        robot_ip=robot_ip,
        dof=7,
        robot_type="xarm",
        hw_ns="xarm",
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
    ).to_moveit_configs()

    # ── Inject the MTC execution capability into move_group ──────────────
    # move_group/ExecuteTaskSolutionCapability is provided by
    # moveit_task_constructor_capabilities.  It must be in move_group's
    # `capabilities` parameter or task.execute() will fail with error 99999.
    moveit_config_dict = moveit_config.to_dict()
    existing_caps = moveit_config_dict.get("capabilities", "")
    mtc_cap = "move_group/ExecuteTaskSolutionCapability"
    if mtc_cap not in existing_caps:
        moveit_config_dict["capabilities"] = (
            (existing_caps + " " + mtc_cap).strip()
        )

    # ── Robot description (URDF → robot_state_publisher) ─────────────────
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("xarm_description"),
            "launch",
            "_robot_description.launch.py",
        ])),
        launch_arguments={
            "robot_description": yaml.dump(moveit_config.robot_description),
        }.items(),
    )

    # ── move_group with MTC capability ────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    # ── ros2_control (fake or real hardware interface) ────────────────────
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("xarm_controller"),
            "launch",
            "_ros2_control.launch.py",
        ])),
        launch_arguments={
            "robot_description": yaml.dump(moveit_config.robot_description),
            "ros2_control_params": ros2_control_params,
        }.items(),
    )

    # ── Controller spawners ───────────────────────────────────────────────
    joint_state_broadcaster = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )],
    )

    xarm7_controller = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["xarm7_traj_controller", "--controller-manager", "/controller_manager"],
        )],
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare("xarm_moveit_config"), "rviz", "moveit.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{
            "robot_description":            moveit_config_dict["robot_description"],
            "robot_description_semantic":   moveit_config_dict["robot_description_semantic"],
            "robot_description_kinematics": moveit_config_dict["robot_description_kinematics"],
            "robot_description_planning":   moveit_config_dict["robot_description_planning"],
            "planning_pipelines":           moveit_config_dict["planning_pipelines"],
        }],
    )

    # ── Static TF: world → link_base ─────────────────────────────────────
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "link_base"],
    )

    # ── Gripper node ──────────────────────────────────────────────────────
    gripper_node = Node(
        package="gripper_pkg",
        executable="gripper_node",
        name="gripper_node",
        parameters=[config, {"sim_mode": sim_mode}],
    )

    # ── MTC pick-and-place node (replaces planner_node / deterministic_planner_node) ──
    # Both robot_description* params AND our own params are passed so that
    # task.loadRobotModel() finds the SRDF on this node directly.
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
                        "sim_mode":  sim_mode,
                        "ee_link":   ee_link_val,
                        "sim_pot_x": sim_pot_x,
                        "sim_pot_y": sim_pot_y,
                        "sim_pot_z": sim_pot_z,
                    },
                ],
            )
        ],
    )

    # ── Perception node (optional, uncomment to enable) ───────────────────
    perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pot_detector'),
                'launch',
                'pot_detection.launch.py'
            )
        )
    )

    return [
        robot_description_launch,
        static_tf,
        ros2_control_launch,
        move_group_node,
        joint_state_broadcaster,
        xarm7_controller,
        rviz_node,
        gripper_node,
        mtc_node,
        perception_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_mode",  default_value="false",
                              description="Use fake hardware (no robot IP needed)"),
        DeclareLaunchArgument("robot_ip",  default_value="192.168.1.201",
                              description="xArm controller IP (real robot only)"),
        DeclareLaunchArgument("ee_link",   default_value="",
                              description="EEF link for IK (default: link_eef in sim, tool_tcp on real robot)"),
        DeclareLaunchArgument("sim_pot_x", default_value="0.0"),
        DeclareLaunchArgument("sim_pot_y", default_value="-0.9"),
        DeclareLaunchArgument("sim_pot_z", default_value="0.160"),
        OpaqueFunction(function=launch_setup),
    ])
