# launch/planting.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='planting_controller',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200
            }]
        ),

        # Node(
        #     package='planting_controller',
        #     executable='planting_fsm',
        #     name='planting_fsm',
        #     output='screen'
        # ),
        
        # add a manual ndoe to test the serial bridge and the arudino without the actual fsm node sending arduino commands
        Node(
            package='planting_controller',
            executable='manual_fsm_tester',
            name = 'manual_fsm_tester',
            output='screen'
        )

    ])