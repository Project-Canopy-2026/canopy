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

Every number worth changing in the field is a launch argument, so a depth or
threshold change needs no rebuild:

    ros2 launch planting_controller planting_bringup.launch.py \
        drilling_distance_cm:=20.0 fsr_abort_voltage:=3.5

    ros2 launch planting_controller planting_bringup.launch.py \
        arduino_port:=/dev/ttyACM1 drilling_dwell:=2.0

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
# Defaults mirror the in-code defaults of each node, so launching with no
# arguments behaves exactly as running the three nodes by hand.
_ARGS = [
    # Arduino / serial
    ('arduino_port',  '/dev/ttyACM0', 'Arduino serial port. A udev rule for '
                                      '/dev/arduino exists — see note below.'),
    ('arduino_baud',  '115200',       'Arduino baud rate; must match '
                                      'planting_arduino.ino Serial.begin()'),
    # CAN / LINAK
    ('can_channel',   'can1',         'SocketCAN interface the LINAKs are on'),
    ('can_bitrate',   '125000',       'LINAK CAN bitrate'),
    # Motion
    ('drilling_distance_cm', '30.0',  'Drill depth, cm from home. Clamped to the '
                                      '30 cm stroke by the driver'),
    ('drilling_dwell',       '5.0',   'Seconds spinning at the bottom of the hole'),
    ('chute_distance_cm',    '20.0',  'Chute depth, cm from home'),
    ('shift_distance_cm',    '23.0',  'Stepper travel auger ↔ chute (cm, measured)'),
    ('auger_rpm',            '75',    'BLDC auger speed (RPM)'),
    # Obstacle abort
    ('fsr_abort_voltage',    '3.7',   'FSR voltage that counts as hitting '
                                      'something. UNCALIBRATED — see WORKFLOW.md'),
    ('fsr_abort_seconds',    '2.0',   'How long it must hold there before aborting'),
    ('fsr_poll_hz',          '10.0',  'How often the FSR is read while drilling'),
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
# so the port can be given as arduino_port:=/dev/arduino and stop depending on
# whether the board enumerated as ACM0 or ACM1.
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
    parameters=[{
        'drilling_distance_cm': _param('drilling_distance_cm', float),
        'drilling_dwell':       _param('drilling_dwell',       float),
        'chute_distance_cm':    _param('chute_distance_cm',    float),
        'shift_distance_cm':    _param('shift_distance_cm',    float),
        'auger_rpm':            _param('auger_rpm',            int),
        'fsr_abort_voltage':    _param('fsr_abort_voltage',    float),
        'fsr_abort_seconds':    _param('fsr_abort_seconds',    float),
        'fsr_poll_hz':          _param('fsr_poll_hz',          float),
    }],
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
