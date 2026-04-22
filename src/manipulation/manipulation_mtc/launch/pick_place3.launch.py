from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    sim_mode = LaunchConfiguration("sim_mode")
    ee_link = LaunchConfiguration("ee_link")

    return LaunchDescription([

        # ── Args ─────────────────────────────────────────────
        DeclareLaunchArgument(
            "sim_mode",
            default_value="true",
            description="Run in simulation mode with fake pot input"
        ),

        DeclareLaunchArgument(
            "ee_link",
            default_value="tool_tcp",
            description="End effector link name"
        ),

        # ── Pick and Place Node ──────────────────────────────
        Node(
            package="manipulation_mtc",   
            executable="pick_and_place_node",
            name="pick_and_place",
            output="screen",
            emulate_tty=True,

            parameters=[{
                "sim_mode": sim_mode,
                "ee_link": ee_link,

                # optional sim pot override
                "sim_pot_x": 0.0,
                "sim_pot_y": -0.35,
                "sim_pot_z": 0.16,
            }],

            # good for debugging MTC failures
            arguments=["--ros-args", "--log-level", "info"],
        ),
    ])