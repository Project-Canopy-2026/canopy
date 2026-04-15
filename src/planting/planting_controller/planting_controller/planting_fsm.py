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
    LINAK_HOME      = 'LINAK_HOME'      # both linaks retract to IN_MAX [transition: 5s timer]
    AUGER_SPIN_UP   = 'AUGER_SPIN_UP'   # bldc continuous spin [transition: immediate]
    DRILLING_DOWN   = 'DRILLING_DOWN'   # linak runs out to max position 64255[transition: DONE:LINAK1]
    DRILLING_DWELL  = 'DRILLING_DWELL'  # bldc continuous spin [transition: 10s timer]
    AUGER_RETRACT   = 'AUGER_RETRACT'   # bldc spins other way, linak runs in to max position 150 [transition: DONE:LINAK1]
    SHIFT_TO_CHUTE  = 'SHIFT_TO_CHUTE'  # bldc stops, stepper moves shift_distance_cm [transition: DONE:STEPPER → publish /chute_in_position]
    WAIT_SEEDLING   = 'WAIT_SEEDLING'   # wait for seedling_dropped topic [transition: seedling_dropped]
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
      /behavior/do_plant (std_msgs/Empty) — triggers the sequence from IDLE
      seedling_dropped (std_msgs/Bool)   — True advances WAIT_SEEDLING
      /arduino_status  (std_msgs/String) — ACK/DONE/ERR from Arduino via serial_bridge
      /linak_status    (std_msgs/String) — ACK/DONE/ERR from linak_can_node

    Publications:
      /arduino_cmd     (std_msgs/String) — commands to Arduino
      /linak_cmd       (std_msgs/String) — commands to LINAK actuators
      planting_state   (std_msgs/String) — current FSM state name

    Parameters:
      drilling_distance_cm  float  6.0   LINAK_1 down distance (cm)
      retract_distance_cm   float  6.0   LINAK_1 up distance (cm)
      drilling_dwell        float  10.0  dwell in soil (s)
      chute_distance_cm     float  6.0   LINAK_2 down/up distance (cm)
      shift_distance_cm     float  5.0   stepper travel distance (cm) — speed locked at 270 RPM in firmware
      auger_rpm             int    75    BLDC RPM
      (LINAK speed assumed constant at 2.18 cm/s — duration = distance / 2.18)
    """

    def __init__(self):
        super().__init__('planting_fsm')

        # ── OUTDOOR PARAMS ────────────────────────────────────────────────────
        self.declare_parameter('drilling_distance_cm', 30.0) # tbd
        self.declare_parameter('retract_distance_cm',  30.0) # tbd
        self.declare_parameter('drilling_dwell',       5.0) # tbd
        self.declare_parameter('chute_distance_cm',    20.0) # tbd
        self.declare_parameter('shift_distance_cm',    23.0) # measured
        self.declare_parameter('auger_rpm',            75)

        # ── INDOOR PARAMS ────────────────────────────────────────────────────
        # self.declare_parameter('drilling_distance_cm', 12.0) # tbd
        # self.declare_parameter('retract_distance_cm',  12.0) # tbd
        # self.declare_parameter('drilling_dwell',       5.0) # tbd
        # self.declare_parameter('chute_distance_cm',    12.0) # tbd
        # self.declare_parameter('shift_distance_cm',    23.0) # measured
        # self.declare_parameter('auger_rpm',            75)
        # ── Publishers ────────────────────────────────────────────────────
        self._arduino_pub    = self.create_publisher(String, '/arduino_cmd',        10)
        self._linak_pub      = self.create_publisher(String, '/linak_cmd',          10)
        self._state_pub      = self.create_publisher(String, 'planting_state',      10)
        self._chute_pos_pub  = self.create_publisher(Bool,   '/chute_in_position',  10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Empty,  '/behavior/do_planting', self._on_do_planting,  10)
        self.create_subscription(Bool,   '/behavior/seedling_dropped', self._on_seedling,        10)
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
        self._enter(State.LINAK_HOME)

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
                self._chute_pos_pub.publish(Bool(data=True))
                self.get_logger().info('→ /chute_in_position: True')
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
        LINAK_SPEED_CM_S = 2.18   # cm/s — used to convert distance → duration
        LINAK_SPEED_CM_S = 1.09
        p         = self.get_parameter
        auger_rpm = p('auger_rpm').get_parameter_value().integer_value

        if state == State.LINAK_HOME:
            # shift_mm = p('shift_distance_cm').get_parameter_value().double_value * 10.0
            # shift_mm = 130.0
            # self._arduino(f'stepper,left,{shift_mm:.1f}')
            self._linak('LINAK,1,IN_MAX')
            self._linak('LINAK,2,IN_MAX')
            self.get_logger().info('Waiting 5 s for LINAKs to home...')
            with self._lock:
                self._dwell_timer = self.create_timer(10.0, self._homing_done)
            # advances via _homing_done → AUGER_SPIN_UP

        elif state == State.AUGER_SPIN_UP:
            self._arduino(f'bldc,in,{auger_rpm}')
            self._enter(State.DRILLING_DOWN)        # immediate

        elif state == State.DRILLING_DOWN:
            LINAK_SPEED_CM_S = 1.09
            dur = p('drilling_distance_cm').get_parameter_value().double_value / LINAK_SPEED_CM_S
            self._linak(f'LINAK,1,DOWN,{dur:.2f}')
            # advances on DONE:LINAK1

        elif state == State.DRILLING_DWELL:
            dwell = p('drilling_dwell').get_parameter_value().double_value
            self.get_logger().info(f'Dwelling in soil for {dwell} s...')
            with self._lock:
                self._dwell_timer = self.create_timer(dwell, self._dwell_done)

        elif state == State.AUGER_RETRACT:
            dur = p('retract_distance_cm').get_parameter_value().double_value / LINAK_SPEED_CM_S
            self._arduino(f'bldc,in,{auger_rpm}')
            self._linak(f'LINAK,1,UP,{dur:.2f}')
            # advances on DONE:LINAK1

        elif state == State.SHIFT_TO_CHUTE:
            self._arduino('bldc,stop')
            shift_mm = p('shift_distance_cm').get_parameter_value().double_value * 10.0
            self._arduino(f'stepper,left,{shift_mm:.1f}')
            # advances on DONE:STEPPER → publishes /chute_in_position True → WAIT_SEEDLING

        elif state == State.WAIT_SEEDLING:
            self.get_logger().info('Waiting for seedling_dropped...')

        elif state == State.CHUTE_DOWN:
            dur = p('chute_distance_cm').get_parameter_value().double_value / LINAK_SPEED_CM_S
            self._linak(f'LINAK,2,DOWN,{dur:.2f}')
            # advances on DONE:LINAK2

        elif state == State.CHUTE_RETRACT:
            dur = p('chute_distance_cm').get_parameter_value().double_value / LINAK_SPEED_CM_S
            self._linak(f'LINAK,2,UP,{dur:.2f}')
            # advances on DONE:LINAK2

        elif state == State.SHIFT_TO_AUGER:
            shift_mm = p('shift_distance_cm').get_parameter_value().double_value * 10.0
            self._arduino(f'stepper,right,{shift_mm:.1f}')
            # advances on DONE:STEPPER

        elif state == State.COMPLETE:
            self._arduino('bldc,stop')
            self.get_logger().info('Planting sequence complete.')
            self._enter(State.IDLE)                 # ready for next cycle

        elif state == State.FAULT:
            self._arduino('stop')                   # emergency stop all Arduino motors
            self.get_logger().error('FAULT — emergency stop issued. Restart node to reset.')

    def _homing_done(self):
        with self._lock:
            if self._dwell_timer is not None:
                self._dwell_timer.cancel()
                self._dwell_timer = None
        self._enter(State.AUGER_SPIN_UP)

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
