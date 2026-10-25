# launch/planting_bringup.launch.py
"""
Planting bring-up — serial_bridge, then linak_can_node, then planting_fsm.

Each node waits for the previous one to report ready, because starting the FSM
before the Arduino has sent READY means its first bldc command is dropped on
the floor by serial_bridge, and starting it before the actuators are
OPERATIONAL means the first RPDO goes nowhere:

    serial_bridge  --"Arduino is READY"-->  linak_can_node
    linak_can_node --"both actuators initialised"-->  planting_fsm

The interactive tester is NOT started here — it needs a real stdin, which a
launched node does not get. Run it in its own terminal:

    ros2 run planting_controller planting_fsm_test

FSM tuning (depths, dwell, RPM, FSR abort) is set ONLY in planting_fsm.py
(the INDOOR / OUTDOOR PARAMS block in __init__). This file deliberately does
not pass those parameters, so there is one place to change them. With
--symlink-install an edit there takes effect on the next launch.

Hardware settings remain launch arguments:

    ros2 launch planting_controller planting_bringup.launch.py \
        arduino_port:=/dev/ttyACM1 can_channel:=can0

See `ros2 launch planting_controller planting_bringup.launch.py -s` for the
full list with defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ── Launch arguments ─────────────────────────────────────────────────────────
# (name, default, description) — declared below and wired into the nodes.
# Hardware only. FSM tuning lives in planting_fsm.py and is not passed here.
_ARGS = [
    # Arduino / serial
    ('arduino_port',  '/dev/arduino', 'Arduino serial port. /dev/arduino is a '
                                      'udev symlink — see note below.'),
    ('arduino_baud',  '115200',       'Arduino baud rate; must match '
                                      'planting_arduino.ino Serial.begin()'),
    # CAN / LINAK
    ('can_channel',   'can1',         'SocketCAN interface the LINAKs are on'),
    ('can_bitrate',   '125000',       'LINAK CAN bitrate'),
]


def _declare():
    return [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in _ARGS
    ]


def _param(name, value_type):
    """
    A launch argument as a correctly typed ROS parameter.

    Substitutions resolve to strings, and the nodes declare these as double /
    integer, so handing the raw LaunchConfiguration straight over gets the
    parameter rejected at startup. ParameterValue does the conversion.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _on_output(event: ProcessIO, trigger_str: str, actions, latch: dict):
    """
    Return `actions` the first time `trigger_str` shows up in this process's
    output, and never again — OnProcessIO fires per line, so without the latch
    a repeated banner would start a second copy of the node.
    """
    if latch.get(trigger_str):
        return []
    if trigger_str in event.text.decode(errors='ignore'):
        latch[trigger_str] = True
        return actions
    return []


# I created the rule at /etc/udev/rules.d/99-arduino.rules:
#   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="arduino"
# and applied it with:
#   sudo udevadm control --reload-rules && sudo udevadm trigger
# so the default port is /dev/arduino and does not depend on whether the board
# enumerated as ACM0 or ACM1. Pass arduino_port:=/dev/ttyACMx to override.
serial_bridge = Node(
    package='planting_controller',
    executable='serial_bridge',
    name='serial_bridge',
    output='screen',
    emulate_tty=True,          # line-buffered, so READY arrives promptly
    parameters=[{
        'port':     _param('arduino_port', str),
        'baudrate': _param('arduino_baud', int),
    }],
)

linak_can_node = Node(
    package='planting_controller',
    executable='linak_can_node',
    name='linak_can_node',
    output='screen',
    emulate_tty=True,
    parameters=[{
        'can_channel': _param('can_channel', str),
        'can_bitrate': _param('can_bitrate', int),
    }],
)

planting_fsm = Node(
    package='planting_controller',
    executable='planting_fsm',
    name='planting_fsm',
    output='screen',
    emulate_tty=True,
    # No parameters: planting_fsm.py's declare_parameter defaults are the
    # only source of FSM tuning.
)


def generate_launch_description():
    latch = {}

    return LaunchDescription([
        *_declare(),

        # 1. Serial bridge — opens the port, waits for the Arduino's READY.
        serial_bridge,

        # 2. On "Arduino is READY", bring up the CAN driver.
        RegisterEventHandler(
            OnProcessIO(
                target_action=serial_bridge,
                on_stdout=lambda e: _on_output(e, 'Arduino is READY',
                                               [linak_can_node], latch),
                on_stderr=lambda e: _on_output(e, 'Arduino is READY',
                                               [linak_can_node], latch),
            )
        ),

        # 3. On "both actuators initialised", start the FSM. It then sits in
        #    IDLE until something publishes /behavior/do_planting.
        RegisterEventHandler(
            OnProcessIO(
                target_action=linak_can_node,
                on_stdout=lambda e: _on_output(e, 'both actuators initialised',
                                               [planting_fsm], latch),
                on_stderr=lambda e: _on_output(e, 'both actuators initialised',
                                               [planting_fsm], latch),
            )
        ),
    ])
