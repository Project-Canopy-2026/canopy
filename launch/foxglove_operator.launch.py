"""Foxglove operator station: foxglove_bridge + canopy_launcher.

Start this INSTEAD of canopy_launch.launch.py when you want to drive the robot
from the Foxglove "Seedling Planner" tab:

    ros2 launch <repo>/launch/foxglove_operator.launch.py

Then in Foxglove (ws://<jetson-ip>:8765), tab 3:
    1. "Start planting" -> canopy_launcher runs canopy_launch.launch.py foxglove:=false
    2. pick or paste seedlings
    3. "Send plan" -> /planning/requested_mode {level: 2}, then /planning/complete_plan

Do not also run canopy_launch.launch.py by hand: it includes viz.launch.py,
which starts a second foxglove_bridge on the same port.

The stack's output is printed in this terminal (and saved to
~/.ros/canopy_launcher/canopy_<time>.log). foxglove_bridge only prints warnings.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    launch_command = LaunchConfiguration("launch_command")

    # Started directly (not via foxglove_bridge_launch.xml) so it can run at WARN:
    # at INFO it logs every client connect and subscribe, which buries the stack's
    # output in this terminal. Its full log is still in ~/.ros/log.
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765, "address": "0.0.0.0"}],
        ros_arguments=["--log-level", "warn"],
        output="screen",
    )

    launcher = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(REPO_ROOT, "scripts", "canopy_launcher.py"),
            "--ros-args",
            "-p",
            ["launch_command:=", launch_command],
        ],
        name="canopy_launcher",
        output="screen",
        # The launcher prints the stack's output, which already names each node;
        # don't add a "[canopy_launcher-N]" prefix in front of every line.
        output_format="{line}",
        # Give the launcher time to Ctrl-C the robot stack before ros2 launch
        # escalates to SIGTERM / SIGKILL on shutdown.
        sigterm_timeout="30",
        sigkill_timeout="10",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_command",
            default_value=(
                "ros2 launch "
                + os.path.join(REPO_ROOT, "launch", "canopy_launch.launch.py")
                + " foxglove:=false"
            ),
            description="Command the Start planting button runs",
        ),
        foxglove_bridge,
        launcher,
    ])
