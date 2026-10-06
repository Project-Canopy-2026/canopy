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
    LINAK_HOME      = 'LINAK_HOME'      # both linaks HOME [transition: 5s timer]
    AUGER_SPIN_UP   = 'AUGER_SPIN_UP'   # bldc continuous spin [transition: immediate]
    DRILLING_DOWN   = 'DRILLING_DOWN'   # linak_1 GOTO drilling_distance_cm [transition: DONE:LINAK1]
    DRILLING_DWELL  = 'DRILLING_DWELL'  # bldc continuous spin [transition: drilling_dwell timer]
    AUGER_RETRACT   = 'AUGER_RETRACT'   # bldc still spinning, linak_1 HOME [transition: DONE:LINAK1]
    SHIFT_TO_CHUTE  = 'SHIFT_TO_CHUTE'  # bldc stops, stepper moves shift_distance_cm [transition: DONE:STEPPER → publish /chute_in_position]
    WAIT_SEEDLING   = 'WAIT_SEEDLING'   # wait for seedling_dropped topic [transition: seedling_dropped]
    CHUTE_DOWN      = 'CHUTE_DOWN'      # linak_2 GOTO chute_distance_cm [transition: DONE:LINAK2 or STALL]
    CHUTE_RETRACT   = 'CHUTE_RETRACT'   # linak_2 HOME [transition: DONE:LINAK2]
    SHIFT_TO_AUGER  = 'SHIFT_TO_AUGER'  # stepper returns [transition: DONE:STEPPER]
    COMPLETE        = 'COMPLETE'
    DIGGING_OBSTACLE = 'DIGGING_OBSTACLE'  # FSR over threshold or drill stalled: stop, HOME, skip this seedling [transition: STOPPED then DONE:LINAK1]
    FAULT           = 'FAULT'


# States where the auger is in the ground and the FSR is worth watching.
_FSR_MONITORED_STATES = (State.DRILLING_DOWN, State.DRILLING_DWELL)

# Terminal move outcomes linak_can_node reports, each as
# '<OUTCOME>:LINAK<id>,<travelled_cm>'.
_LINAK_OUTCOMES = ('DONE', 'STALL', 'STOPPED')


def _parse_linak_status(token: str):
    """'DONE:LINAK1,15.32' → ('DONE', 1, 15.32). Returns (None, None, None) if it isn't one."""
    head, _, travelled_str = token.partition(',')
    outcome, _, target = head.partition(':')
    if outcome not in _LINAK_OUTCOMES or not target.startswith('LINAK'):
        return None, None, None
    try:
        lid = int(target[5:])
    except ValueError:
        return None, None, None
    try:
        travelled = float(travelled_str)
    except ValueError:
        travelled = 0.0     # tolerate a token without the distance field
    return outcome, lid, travelled


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
      drilling_distance_cm  float  30.0  LINAK_1 drill depth from home (cm)
      drilling_dwell        float  5.0   dwell in soil (s)
      chute_distance_cm     float  20.0  LINAK_2 chute depth from home (cm)
      shift_distance_cm     float  5.0   stepper travel distance (cm) — speed locked at 270 RPM in firmware
      auger_rpm             int    75    BLDC RPM
      fsr_abort_voltage     float  3.7   abort the drill above this FSR voltage
      fsr_poll_hz           float  10.0  how often to poll the FSR while drilling
      fsr_abort_seconds     float  2.0   sustained time over threshold before aborting

    Obstacle abort:
      While DRILLING_DOWN or DRILLING_DWELL the node polls the Arduino with
      'fsr,read' and watches the FSR:<v>,<ohms> replies. fsr_abort_seconds of
      sustained voltage at or above fsr_abort_voltage means the auger has met a
      rock or hard surface: the FSM enters DIGGING_OBSTACLE, stops LINAK_1,
      sends it HOME (auger still spinning so it does not jam), stops the auger
      once clear, and returns to IDLE *without* shifting to the chute — nothing
      is planted at an obstructed site. A STALL:LINAK1 during DRILLING_DOWN
      takes the same path, covering obstacles the actuator gives up on before
      the FSR registers them. The threshold is UNCALIBRATED: the 12 initial
      field trials peaked at 3.559 V on workable soil, but no rock/wood strike
      has been logged yet.

      LINAK moves are ABSOLUTE positions in cm from home: 'GOTO,<depth>' for the
      two downward moves, 'HOME' for every retract. The driver clamps each
      target to the stroke, so a command deeper than the actuator can go gives a
      shallower hole rather than a stall against the end stop that would be
      reported as an obstacle. Per-move speed: drilling down SLOW (1.09 cm/s,
      0x64); auger retract and chute down/up FAST (2.18 cm/s, 0xCD).
    """

    def __init__(self):
        super().__init__('planting_fsm')

        # ── OUTDOOR PARAMS ────────────────────────────────────────────────────
        # Distances are absolute depths from home (0 cm = fully retracted).
        # Retracts take no parameter: they go HOME.
        self.declare_parameter('drilling_distance_cm', 28.0)
        self.declare_parameter('drilling_dwell',       6.0)
        self.declare_parameter('chute_distance_cm',    18.0)
        self.declare_parameter('shift_distance_cm',    23.0) # measured
        self.declare_parameter('auger_rpm',            75)

        # FSR obstacle abort. UNCALIBRATED — no rock/wood strike has been logged yet.
        # The 12 initial field trials peaked at 3.559 V on workable soil, so 3.7 V
        # clears every recorded drill; the 3-sample debounce covers the rest of the
        # margin. Retune once a deliberate obstacle test exists.
        self.declare_parameter('fsr_abort_voltage',    2.5)
        self.declare_parameter('fsr_poll_hz',          10.0)
        self.declare_parameter('fsr_abort_seconds',    2.0)

        # ── INDOOR PARAMS ────────────────────────────────────────────────────
        # self.declare_parameter('drilling_distance_cm', 12.0) # tbd
        # self.declare_parameter('drilling_dwell',       5.0) # tbd
        # self.declare_parameter('chute_distance_cm',    12.0) # tbd
        # self.declare_parameter('shift_distance_cm',    23.0) # measured
        # self.declare_parameter('auger_rpm',            75)
        # self.declare_parameter('fsr_abort_voltage',    3.7) # tbd
        # self.declare_parameter('fsr_poll_hz',          10.0)
        # self.declare_parameter('fsr_abort_seconds',    2.0)
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

        # FSR polling gets its own timer slot — _enter() blindly cancels
        # _dwell_timer, which is already shared by the homing and dwell timers.
        self._fsr_timer   = None
        self._fsr_over    = 0       # consecutive samples over threshold

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
        elif token.startswith('FSR:'):
            # Only act while the auger is actually in the ground; a late reply
            # arriving after the drill finished must not abort anything.
            if state in _FSR_MONITORED_STATES:
                self._on_fsr_sample(token)
        elif token.startswith('ERR:'):
            self.get_logger().error(f'Arduino error received: {token}')
            self._enter(State.FAULT)

    def _on_linak_status(self, msg: String):
        token = msg.data.strip()
        with self._lock:
            state = self._state

        if token.startswith('ERR:'):
            self.get_logger().error(f'LINAK error received: {token}')
            self._enter(State.FAULT)
            return

        outcome, lid, travelled = _parse_linak_status(token)
        if outcome is None:
            return

        if lid == 1:
            self._on_linak1(outcome, travelled, state)
        elif lid == 2:
            self._on_linak2(outcome, state)

    def _on_linak1(self, outcome: str, travelled: float, state: State):
        """Auger actuator. DONE/STALL both end a move; STALL means it hit something."""
        if state == State.DIGGING_OBSTACLE:
            if outcome == 'STOPPED':
                # The drill move was aborted mid-travel. Go HOME — an absolute
                # target, so it does not matter how far down it got. (This used
                # to retract by exactly `travelled`, which was correct but only
                # as accurate as that figure; HOME cannot be wrong.) The
                # distance is still logged, because a value well short of the
                # commanded depth is what tells a real obstacle from the end
                # stop.
                self.get_logger().warn(
                    f'Drill aborted after {travelled:.2f} cm — retracting to home.'
                )
                self._linak('LINAK,1,HOME')
            else:
                # Retract finished (DONE, or STALL at the hard stop). The auger
                # is clear of the hole, so it is safe to stop it now. Back to
                # IDLE, not SHIFT_TO_CHUTE — nothing is planted at an
                # obstructed site.
                self._arduino('bldc,stop')
                self.get_logger().warn(
                    'Obstacle abort complete — skipping this seedling.'
                )
                self._enter(State.IDLE)
            return

        if state == State.DRILLING_DOWN:
            if outcome == 'STALL':
                # Stopped short of target depth: the auger is against something
                # solid. Same remedy as an FSR trip, and it covers obstacles
                # too hard to register force before the actuator gives up.
                self.get_logger().error(
                    f'Drill stalled at {travelled:.2f} cm — obstacle.'
                )
                self._enter(State.DIGGING_OBSTACLE)
            elif outcome == 'DONE':
                self._enter(State.DRILLING_DWELL)

        elif state == State.AUGER_RETRACT and outcome in ('DONE', 'STALL'):
            self._enter(State.SHIFT_TO_CHUTE)

        elif state == State.LINAK_HOME and outcome in ('DONE', 'STALL'):
            pass        # homing advances on its own timer

    def _on_linak2(self, outcome: str, state: State):
        """Chute actuator. A stall here is the chute grounding — expected."""
        if outcome not in ('DONE', 'STALL'):
            return
        if state == State.CHUTE_DOWN:
            self._enter(State.CHUTE_RETRACT)
        elif state == State.CHUTE_RETRACT:
            self._enter(State.SHIFT_TO_AUGER)

    # ── State machine ─────────────────────────────────────────────────────

    def _enter(self, new_state: State):
        with self._lock:
            old = self._state
            self._state = new_state
            if self._dwell_timer is not None:
                self._dwell_timer.cancel()
                self._dwell_timer = None

        # Poll the FSR only while the auger is in the ground. Driven from here
        # rather than _on_enter so states without an entry branch (IDLE) still
        # shut the poller down.
        if new_state in _FSR_MONITORED_STATES:
            self._start_fsr_poll()
        else:
            self._stop_fsr_poll()

        self.get_logger().info(f'FSM: {old.value} → {new_state.value}')
        self._state_pub.publish(String(data=new_state.value))
        self._on_enter(new_state)

    # ── FSR obstacle detection ────────────────────────────────────────────

    def _start_fsr_poll(self):
        hz = self.get_parameter('fsr_poll_hz').get_parameter_value().double_value
        with self._lock:
            self._fsr_over = 0
            if self._fsr_timer is not None:
                return                          # already polling, keep it running
            self._fsr_timer = self.create_timer(1.0 / hz, self._poll_fsr)

    def _stop_fsr_poll(self):
        with self._lock:
            self._fsr_over = 0
            if self._fsr_timer is not None:
                self._fsr_timer.cancel()
                self._fsr_timer = None

    def _poll_fsr(self):
        # Arduino replies FSR:<voltage>,<ohms>, handled in _on_arduino_status.
        self._arduino_pub.publish(String(data='fsr,read'))

    def _on_fsr_sample(self, token: str):
        """Count consecutive over-threshold samples; abort the drill on N in a row."""
        try:
            voltage = float(token[4:].split(',')[0])
        except (ValueError, IndexError):
            self.get_logger().warn(f'Malformed FSR reply: {token}')
            return

        p = self.get_parameter
        threshold = p('fsr_abort_voltage').get_parameter_value().double_value
        seconds   = p('fsr_abort_seconds').get_parameter_value().double_value
        hz        = p('fsr_poll_hz').get_parameter_value().double_value

        # Count samples rather than wall-clock: if replies are dropped we
        # under-count and simply take longer to trip, which is the safe
        # direction. A clock would keep running through the silence and could
        # fire on two samples two seconds apart.
        needed = max(1, int(round(seconds * hz)))

        with self._lock:
            if voltage < threshold:
                self._fsr_over = 0
                return
            self._fsr_over += 1
            over    = self._fsr_over
            tripped = over >= needed
            if tripped:
                self._fsr_over = 0

        if not tripped:
            self.get_logger().warn(
                f'FSR {voltage:.3f} V ≥ {threshold:.2f} V ({over}/{needed})'
            )
            return

        self.get_logger().error(
            f'FSR {voltage:.3f} V ≥ {threshold:.2f} V for {seconds:.1f} s — obstacle.'
        )
        self._enter(State.DIGGING_OBSTACLE)

    def _on_enter(self, state: State):
        # LINAK moves are now distance-based (cm). linak_can_node computes the
        # target count from TPDO feedback and stops there. The FAST/SLOW tag
        # only controls hardware travel speed (0xCD vs 0x64 RPDO speed byte).

        p         = self.get_parameter
        auger_rpm = p('auger_rpm').get_parameter_value().integer_value

        if state == State.LINAK_HOME:
            self._linak('LINAK,1,HOME')
            self._linak('LINAK,2,HOME')
            self.get_logger().info('Waiting 5 s for LINAKs to home...')
            with self._lock:
                self._dwell_timer = self.create_timer(5.0, self._homing_done)
            # advances via _homing_done → AUGER_SPIN_UP

        elif state == State.AUGER_SPIN_UP:
            self._arduino(f'bldc,in,{auger_rpm}')
            self._enter(State.DRILLING_DOWN)        # immediate

        elif state == State.DRILLING_DOWN:
            # Drill down slowly so the auger can bite into soil. GOTO is an
            # absolute depth from home, so a homing that finished short makes
            # the hole shallower — it cannot aim past the end stop and stall
            # there, which the driver would have reported as a rock.
            depth = p('drilling_distance_cm').get_parameter_value().double_value
            self._linak(f'LINAK,1,GOTO,{depth:.2f},SLOW')
            # advances on DONE:LINAK1

        elif state == State.DRILLING_DWELL:
            dwell = p('drilling_dwell').get_parameter_value().double_value
            self.get_logger().info(f'Dwelling in soil for {dwell} s...')
            with self._lock:
                self._dwell_timer = self.create_timer(dwell, self._dwell_done)

        elif state == State.AUGER_RETRACT:
            # Retract the auger to home at full speed, still spinning.
            self._arduino(f'bldc,in,{auger_rpm}')
            self._linak('LINAK,1,HOME')
            # advances on DONE:LINAK1

        elif state == State.SHIFT_TO_CHUTE:
            self._arduino('bldc,stop')
            shift_mm = p('shift_distance_cm').get_parameter_value().double_value * 10.0
            self._arduino(f'stepper,left,{shift_mm:.1f}')
            # advances on DONE:STEPPER → publishes /chute_in_position True → WAIT_SEEDLING

        elif state == State.WAIT_SEEDLING:
            self.get_logger().info('Waiting for seedling_dropped...')

        elif state == State.CHUTE_DOWN:
            depth = p('chute_distance_cm').get_parameter_value().double_value
            self._linak(f'LINAK,2,GOTO,{depth:.2f},FAST')
            # advances on DONE:LINAK2 — or STALL, which is the tube grounding

        elif state == State.CHUTE_RETRACT:
            self._linak('LINAK,2,HOME')
            # advances on DONE:LINAK2

        elif state == State.SHIFT_TO_AUGER:
            shift_mm = p('shift_distance_cm').get_parameter_value().double_value * 10.0
            self._arduino(f'stepper,right,{shift_mm:.1f}')
            # advances on DONE:STEPPER

        elif state == State.COMPLETE:
            self._arduino('bldc,stop')
            self.get_logger().info('Planting sequence complete.')
            self._enter(State.IDLE)                 # ready for next cycle

        elif state == State.DIGGING_OBSTACLE:
            # Halt the drill. The driver answers STOPPED:LINAK1,<cm> with how
            # far it actually got, and _on_linak1 then retracts exactly that
            # much — retracting the commanded depth after a short drill would
            # grind the actuator into its own hard stop.
            # The auger keeps spinning until the retract finishes (same as
            # AUGER_RETRACT) so it does not jam stationary in the hole.
            self.get_logger().warn('Obstacle detected — aborting drill.')
            self._linak('LINAK,1,STOP')
            # STOPPED:LINAK1 → retract; its DONE/STALL → stop auger, IDLE

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
