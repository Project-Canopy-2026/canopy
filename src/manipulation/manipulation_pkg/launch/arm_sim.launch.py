"""
Simulation launch file for pick-and-place testing in RViz (no real hardware).

What this starts:
  - xarm7_planner_fake: MoveIt2 + RViz with a simulated xArm7 (no robot IP needed)
  - gripper_node (sim_mode=true): auto-succeeds open/close without serial hardware
  - planner node (sim_mode=true):
      * auto-triggers pick-and-place ~8 s after launch
      * skips chute_in_position wait at the drop step

Planner selection (default: planner_node — vision-based with Pilz LIN):
  ros2 launch manipulation_pkg arm_sim.launch.py                          # vision planner
  ros2 launch manipulation_pkg arm_sim.launch.py planner:=deterministic   # joint-angle planner

Fake pot position (vision planner only, overrides GRASP_POSE defaults):
  ros2 launch manipulation_pkg arm_sim.launch.py sim_pot_x:=0.0 sim_pot_y:=-0.35 sim_pot_z:=0.16
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from manipulation_pkg import arm_config as cfg


def generate_launch_description():

    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='vision',
        description='Planner to use: "vision" (planner_node) or "deterministic" (deterministic_planner_node)',
    )
    sim_pot_x_arg = DeclareLaunchArgument(
        'sim_pot_x', default_value=str(cfg.GRASP_POSE['x']),
        description='Fake pot X position in link_base frame (metres)'
    )
    sim_pot_y_arg = DeclareLaunchArgument(
        'sim_pot_y', default_value=str(cfg.GRASP_POSE['y']),
        description='Fake pot Y position in link_base frame (metres)'
    )
    sim_pot_z_arg = DeclareLaunchArgument(
        'sim_pot_z', default_value=str(cfg.GRASP_POSE['z']),
        description='Fake pot Z position in link_base frame (metres)'
    )

    planner = LaunchConfiguration('planner')

    # xArm7 MoveIt2 with fake (simulated) hardware + RViz
    xarm_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_planner'),
            'launch',
            'xarm7_planner_fake.launch.py'
        ])),
        launch_arguments={'hw_ns': 'xarm'}.items(),
    )

    # Gripper in sim_mode: no serial hardware required
    gripper_node = Node(
        package='gripper_pkg',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        parameters=[{'sim_mode': True}],
    )

    # Vision-based planner (planner_node): Pilz LIN + OMPL Cartesian, dynamic grasp from pot pose
    # Delayed 5 s; internally auto-triggers 3 s later (total ~8 s after launch)
    vision_planner_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='manipulation_pkg',
                executable='planner_node',
                name='manipulation_planner',
                output='screen',
                parameters=[{
                    'sim_mode': True,
                    'ee_link': 'link_eef',
                    'sim_pot_x': LaunchConfiguration('sim_pot_x'),
                    'sim_pot_y': LaunchConfiguration('sim_pot_y'),
                    'sim_pot_z': LaunchConfiguration('sim_pot_z'),
                }],
                condition=IfCondition(PythonExpression(["'", planner, "' == 'vision'"])),
            )
        ]
    )

    # Deterministic planner (deterministic_planner_node): hardcoded joint waypoints, no perception
    deterministic_planner_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='manipulation_pkg',
                executable='deterministic_planner_node',
                name='manipulation_planner',
                output='screen',
                parameters=[{'sim_mode': True}],
                condition=IfCondition(PythonExpression(["'", planner, "' == 'deterministic'"])),
            )
        ]
    )

    return LaunchDescription([
        planner_arg,
        sim_pot_x_arg,
        sim_pot_y_arg,
        sim_pot_z_arg,
        xarm_fake_launch,
        gripper_node,
        vision_planner_node,
        deterministic_planner_node,
    ])
