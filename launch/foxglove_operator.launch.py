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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    launch_command = LaunchConfiguration("launch_command")

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
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