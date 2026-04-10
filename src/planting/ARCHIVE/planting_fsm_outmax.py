# Alternative FSM that uses LINAK OUT_MAX / IN_MAX position commands instead of
# timed DOWN / UP moves. DRILLING_DOWN drives LINAK_1 to full extension (64255)
# and AUGER_RETRACT drives it back to full retraction (150). Completion is
# detected by linak_can_node via TPDO position stabilisation, so no duration
# parameters are needed for those two states.
import enum
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


# ============================================================
#  State definitions
# ============================================================

class State(enum.Enum):
    IDLE            = 'IDLE'
    AUGER_SPIN_UP   = 'AUGER_SPIN_UP'   # bldc continuous spin [transition: immediate]
    DRILLING_DOWN   = 'DRILLING_DOWN'   # linak runs out to max position 64255 [transition: DONE:LINAK1]
    DRILLING_DWELL  = 'DRILLING_DWELL'  # bldc continuous spin [transition: 10s timer]
    AUGER_RETRACT   = 'AUGER_RETRACT'   # bldc spins other way, linak runs in to max position 150 [transition: DONE:LINAK1]
    SHIFT_TO_CHUTE  = 'SHIFT_TO_CHUTE'  # bldc stops, stepper starts [transition: DONE:STEPPER]
    WAIT_SEEDLING   = 'WAIT_SEEDLING'   # wait for seedling_dropped topic
    CHUTE_DOWN      = 'CHUTE_DOWN'      # linak_2 down [transition: DONE:LINAK2]
    CHUTE_RETRACT   = 'CHUTE_RETRACT'   # linak_2 up [transition: DONE:LINAK2]
    SHIFT_TO_AUGER  = 'SHIFT_TO_AUGER'  # stepper returns [transition: DONE:STEPPER]
    COMPLETE        = 'COMPLETE'
    FAULT           = 'FAULT'


# ============================================================
#  FSM Node
# ============================================================

class PlantingFsmNode(Node):
    """
    Orchestrates the full planting sequence.

    Subscriptions:
    TODO: need to verify these two topic names and message types with the actual publishers
      /behavior/do_plant (std_msgs/Empty) — triggers the sequence from IDLE
      seedling_dropped (std_msgs/Bool)   — True advances WAIT_SEEDLING
      /arduino_status  (std_msgs/String) — ACK/DONE/ERR from Arduino via serial_bridge
      /linak_status    (std_msgs/String) — ACK/DONE/ERR from linak_can_node

    Publications:
      /arduino_cmd     (std_msgs/String) — commands to Arduino
      /linak_cmd       (std_msgs/String) — commands to LINAK actuators
      planting_state   (std_msgs/String) — current FSM state name

    Parameters:
      drilling_dwell     float  10.0  dwell in soil (s)
      chute_duration     float  5.0   LINAK_2 down/up time (s)
      shift_duration     float  5.0   stepper shift time (s)
      auger_rpm          int    75    BLDC RPM
      stepper_rpm        int    100   stepper RPM
    """

    def __init__(self):
        super().__init__('planting_fsm')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('drilling_dwell',    10.0)
        self.declare_parameter('chute_duration',    5.0)
        self.declare_parameter('shift_duration',    5.0)
        self.declare_parameter('auger_rpm',         75)
        self.declare_parameter('stepper_rpm',       100)

        # ── Publishers ────────────────────────────────────────────────────
        self._arduino_pub = self.create_publisher(String, '/arduino_cmd',   10)
        self._linak_pub   = self.create_publisher(String, '/linak_cmd',     10)
        self._state_pub   = self.create_publisher(String, 'planting_state', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Empty,  '/behavior/do_plant', self._on_do_planting,  10)
        self.create_subscription(Bool,   'seedling_dropped', self._on_seedling,        10)
        self.create_subscription(String, '/arduino_status',  self._on_arduino_status,  10)
        self.create_subscription(String, '/linak_status',    self._on_linak_status,    10)

        # ── State ─────────────────────────────────────────────────────────
        self._state       = State.IDLE
        self._lock        = threading.Lock()
        self._dwell_timer = None

        self.get_logger().info('planting_fsm ready — waiting for do_planting.')

    # ── Topic callbacks ───────────────────────────────────────────────────

    def _on_do_planting(self, msg: Empty):
        with self._lock:
            if self._state != State.IDLE:
                self.get_logger().warn(
                    f'do_planting received in state {self._state.value} — ignoring.'
                )
                return
        self._enter(State.AUGER_SPIN_UP)

    def _on_seedling(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            if self._state != State.WAIT_SEEDLING:
                return
        self._enter(State.CHUTE_DOWN)

    def _on_arduino_status(self, msg: String):
        token = msg.data.strip()
        with self._lock:
            state = self._state

        if token == 'DONE:STEPPER':
            if state == State.SHIFT_TO_CHUTE:
                self._enter(State.WAIT_SEEDLING)
            elif state == State.SHIFT_TO_AUGER:
                self._enter(State.COMPLETE)
        elif token.startswith('ERR:'):
            self.get_logger().error(f'Arduino error received: {token}')
            self._enter(State.FAULT)

    def _on_linak_status(self, msg: String):
        token = msg.data.strip()
        with self._lock:
            state = self._state

        if token == 'DONE:LINAK1':
            if state == State.DRILLING_DOWN:
                self._enter(State.DRILLING_DWELL)
            elif state == State.AUGER_RETRACT:
                self._enter(State.SHIFT_TO_CHUTE)

        elif token == 'DONE:LINAK2':
            if state == State.CHUTE_DOWN:
                self._enter(State.CHUTE_RETRACT)
            elif state == State.CHUTE_RETRACT:
                self._enter(State.SHIFT_TO_AUGER)

        elif token.startswith('ERR:'):
            self.get_logger().error(f'LINAK error received: {token}')
            self._enter(State.FAULT)

    # ── State machine ─────────────────────────────────────────────────────

    def _enter(self, new_state: State):
        with self._lock:
            old = self._state
            self._state = new_state
            if self._dwell_timer is not None:
                self._dwell_timer.cancel()
                self._dwell_timer = None

        self.get_logger().info(f'FSM: {old.value} → {new_state.value}')
        self._state_pub.publish(String(data=new_state.value))
        self._on_enter(new_state)

    def _on_enter(self, state: State):
        p            = self.get_parameter
        auger_rpm    = p('auger_rpm').get_parameter_value().integer_value
        stepper_rpm  = p('stepper_rpm').get_parameter_value().integer_value

        if state == State.AUGER_SPIN_UP:
            self._arduino(f'bldc,in,{auger_rpm}')
            self._enter(State.DRILLING_DOWN)        # immediate

        elif state == State.DRILLING_DOWN:
            self._linak('LINAK,1,OUT_MAX')
            # advances on DONE:LINAK1 (linak_can_node detects position stabilisation)

        elif state == State.DRILLING_DWELL:
            dwell = p('drilling_dwell').get_parameter_value().double_value
            self.get_logger().info(f'Dwelling in soil for {dwell} s...')
            with self._lock:
                self._dwell_timer = self.create_timer(dwell, self._dwell_done)

        elif state == State.AUGER_RETRACT:
            self._arduino(f'bldc,out,{auger_rpm}')
            self._linak('LINAK,1,IN_MAX')
            # advances on DONE:LINAK1 (linak_can_node detects position stabilisation)

        elif state == State.SHIFT_TO_CHUTE:
            shift = p('shift_duration').get_parameter_value().double_value
            self._arduino('bldc,stop')
            self._arduino(f'stepper,left,{stepper_rpm},{int(shift)}')
            # advances on DONE:STEPPER

        elif state == State.WAIT_SEEDLING:
            self.get_logger().info('Waiting for seedling_dropped...')

        elif state == State.CHUTE_DOWN:
            dur = p('chute_duration').get_parameter_value().double_value
            self._linak(f'LINAK,2,DOWN,{dur}')
            # advances on DONE:LINAK2

        elif state == State.CHUTE_RETRACT:
            dur = p('chute_duration').get_parameter_value().double_value
            self._linak(f'LINAK,2,UP,{dur}')
            # advances on DONE:LINAK2

        elif state == State.SHIFT_TO_AUGER:
            shift = p('shift_duration').get_parameter_value().double_value
            self._arduino(f'stepper,right,{stepper_rpm},{int(shift)}')
            # advances on DONE:STEPPER

        elif state == State.COMPLETE:
            self._arduino('bldc,stop')
            self.get_logger().info('Planting sequence complete.')
            self._enter(State.IDLE)                 # ready for next cycle

        elif state == State.FAULT:
            self._arduino('stop')                   # emergency stop all Arduino motors
            self.get_logger().error('FAULT — emergency stop issued. Restart node to reset.')

    def _dwell_done(self):
        # ROS2 timers repeat — cancel immediately after first fire
        with self._lock:
            if self._dwell_timer is not None:
                self._dwell_timer.cancel()
                self._dwell_timer = None
        self._enter(State.AUGER_RETRACT)

    # ── Publish helpers ───────────────────────────────────────────────────

    def _arduino(self, cmd: str):
        self._arduino_pub.publish(String(data=cmd))
        self.get_logger().info(f'→ /arduino_cmd: {cmd}')

    def _linak(self, cmd: str):
        self._linak_pub.publish(String(data=cmd))
        self.get_logger().info(f'→ /linak_cmd: {cmd}')


# ============================================================
#  Entry point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = PlantingFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
