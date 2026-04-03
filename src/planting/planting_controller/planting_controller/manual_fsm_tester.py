#!/usr/bin/env python3
# ============================================================
#  Manual FSM Tester — planting_controller
#
#  Interactive CLI to manually step through the planting FSM
#  and send Arduino commands, for testing the serial_bridge
#  and Arduino WITHOUT the LINAK actuator connected.
#
#  Subscribes:  /arduino_status (std_msgs/String) — prints incoming lines
#  Publishes:   /arduino_cmd    (std_msgs/String) — forwards to Arduino
#
#  Launch:
#    ros2 run planting_controller manual_fsm_tester
#    OR via planting_bringup.launch.py (alongside serial_bridge)
#
#  CLI:
#    <raw cmd>              e.g. BLDC,IN,50       → /arduino_cmd
#    step                                          → advance FSM one state
#    reset                                         → return to IDLE
#    state                                         → print current state
#    help                                          → print this list
#    quit / exit                                   → shutdown
# ============================================================

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from planting_controller.planting_fsm import State


# ── FSM state sequence ────────────────────────────────────────────────────────

_STATE_SEQ = [
    State.IDLE,
    State.AUGER_SPIN_UP,
    State.DRILLING_DOWN,
    State.DRILLING_DWELL,
    State.AUGER_RETRACT,
    State.SHIFT_TO_CHUTE,
    State.CHUTE_DOWN,
    State.WAIT_SEEDLING,
    State.CHUTE_RETRACT,
    State.SHIFT_TO_AUGER,
    State.COMPLETE,
]

# Default Arduino command sent on entering each state.
# None = no Arduino command (LINAK-only state or wait state).
_ENTRY_CMD = {
    State.IDLE:           None,
    State.AUGER_SPIN_UP:  'bldc,in,75',
    State.DRILLING_DOWN:  None,           # LINAK vertical down
    State.D
     RILLING_DWELL: None,           # dwell — no new command
    State.AUGER_RETRACT:  'bldc,out,75',  # spin out briefly, then stop
    State.SHIFT_TO_CHUTE: ('bldc,stop','stepper,left,100,4'),
    State.WAIT_SEEDLING:  None,           # operator places seedling
    State.CHUTE_DOWN:     None,           # LINAK vertical down
    State.CHUTE_RETRACT:  None,           # LINAK vertical up
    State.SHIFT_TO_AUGER: 'stepper,right,100,4',
    State.COMPLETE:       'bldc,stop',
}

# States that depend on LINAK — logged as skipped when LINAK is not connected.
_LINAK_STATES = {
    State.DRILLING_DOWN,
    State.AUGER_RETRACT,
    State.CHUTE_DOWN,
    State.CHUTE_RETRACT,
}

_HELP = """\
--- manual_fsm_tester ---
FSM commands:
  step          advance one FSM state, send the default Arduino command
  reset         return FSM to IDLE
  state         print current FSM state

Raw Arduino commands (sent directly to /arduino_cmd):
  bldc,in,<rpm>             spin auger CW  (rpm 0–75)
  bldc,out,<rpm>            spin auger CCW
  bldc,stop                 stop auger
  stepper,left,<rpm>,<s>    move stepper left  for <s> seconds
  stepper,right,<rpm>,<s>   move stepper right for <s> seconds
  stepper,stop              stop stepper immediately
  stop                      emergency stop — halts BLDC and stepper immediately

Other:
  help          show this message
  quit / exit   shutdown
-------------------------"""


class ManualFsmTester(Node):

    def __init__(self):
        super().__init__('manual_fsm_tester')

        # publish to arduino the string commands
        self._cmd_pub = self.create_publisher(String, '/arduino_cmd', 10)

        # listen to serial bridge bridging through arduino status message back
        self._status_sub = self.create_subscription(
            String, '/arduino_status', self._on_status, 10
        )

        self._state_idx = 0
        self._lock = threading.Lock()

        self._cli_thread = threading.Thread(
            target=self._cli_loop, daemon=True, name='cli'
        )
        self._cli_thread.start()

        self.get_logger().info('manual_fsm_tester ready — type "help" for commands.')

    # ── Arduino status subscriber ─────────────────────────────────────────────

    def _on_status(self, msg: String):
        print(f'\n[ARDUINO] {msg.data}', flush=True)

    # ── Arduino command publisher ─────────────────────────────────────────────

    def _send(self, cmd: str):
        msg = String()
        msg.data = cmd.strip()
        self._cmd_pub.publish(msg)
        print(f'[SENT]{msg.data}', flush=True)

    # ── FSM helpers ───────────────────────────────────────────────────────────

    def _current_state(self) -> State:
        with self._lock:
            return _STATE_SEQ[self._state_idx]

    def _step(self):
        with self._lock:
            if self._state_idx >= len(_STATE_SEQ) - 1:
                print('[FSM] Already at COMPLETE — use "reset" to restart.', flush=True)
                return
            self._state_idx += 1
            new_state = _STATE_SEQ[self._state_idx]

        if new_state in _LINAK_STATES:
            print(
                f'[FSM] → {new_state.value}  (LINAK required — not connected, skipping)',
                flush=True,
            )
        else:
            print(f'[FSM] → {new_state.value}', flush=True)

        cmd = _ENTRY_CMD.get(new_state)
        if isinstance(cmd, (list, tuple)):
            for c in cmd:
                self._send(c)
                time.sleep(0.1)
        elif cmd:
            self._send(cmd)

    def _reset(self):
        with self._lock:
            self._state_idx = 0
        print('[FSM] Reset → IDLE', flush=True)

    # ── CLI loop (background thread) ─────────────────────────────────────────

    def _cli_loop(self):
        print(_HELP, flush=True)
        while rclpy.ok():
            try:
                line = input('\n> ').strip()
            except EOFError:
                break

            if not line:
                continue

            lower = line.lower()

            if lower in ('quit', 'exit'):
                print('[INFO] Shutting down...', flush=True)
                rclpy.shutdown()
                break
            elif lower == 'help':
                print(_HELP, flush=True)
            elif lower == 'state':
                print(f'[FSM] Current state: {self._current_state().value}', flush=True)
            elif lower == 'step':
                self._step()
            elif lower == 'reset':
                self._reset()
            else:
                # Treat as a raw Arduino command
                self._send(line)


# ============================================================
#  Entry point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = ManualFsmTester()
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
