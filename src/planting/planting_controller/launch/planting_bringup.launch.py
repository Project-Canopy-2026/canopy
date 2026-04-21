# launch/planting_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO


def on_output(event: ProcessIO, trigger_str: str, actions):
    text = event.text.decode(errors='ignore')
    if trigger_str in text:
        return actions
    return []

# I created  the rule at /etc/udev/rules.d/99-arduino.rules: SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="arduino" and 
# applied it with "sudo udevadm control --reload-rules && sudo udevadm trigger"
# so the parameters can be parameters=[{'port': '/dev/arduino', 'baudrate': 115200}] for planting arduino
serial_bridge = Node(
    package='planting_controller',
    executable='serial_bridge',
    name='serial_bridge',
    output='screen',
    parameters=[{
        # 'port': '/dev/arduino',
        'port': '/dev/ttyACM0',
        'baudrate': 115200
    }]
)

linak_can_node = Node(
    package='planting_controller',
    executable='linak_can_node',
    name='linak_can_node',
    output='screen'
)

planting_fsm = Node(
    package='planting_controller',
    executable='planting_fsm',
    name='planting_fsm',
    output='screen'
)

planting_fsm_test = Node(
    package='planting_controller',
    executable='planting_fsm_test',
    name='planting_fsm_test',
    output='screen'
)


def generate_launch_description():
    return LaunchDescription([

        # 1. Start serial bridge
        serial_bridge,

        # 2. When serial_bridge prints "Arduino is READY", start linak_can_node
        RegisterEventHandler(
            OnProcessIO(
                target_action=serial_bridge,
                on_stdout=lambda event: on_output(event, 'Arduino is READY', [linak_can_node]),
                on_stderr=lambda event: on_output(event, 'Arduino is READY', [linak_can_node]),
            )
        ),

        # 3. When linak_can_node prints "Both LINAK actuators initialised", start FSM + test node
        RegisterEventHandler(
            OnProcessIO(
                target_action=linak_can_node,
                on_stdout=lambda event: on_output(event, 'both actuators initialised', [planting_fsm]),
                on_stderr=lambda event: on_output(event, 'both actuators initialised', [planting_fsm]),
            )
        ),

    ])