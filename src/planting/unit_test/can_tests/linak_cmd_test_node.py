#!/usr/bin/env python3
"""
linak_cmd_test_node.py — interactive CLI tester for linak_can_node.

Run alongside linak_can_node:
  ros2 run planting_controller linak_can_node &
  python3 linak_cmd_test_node.py

Command syntax
──────────────
  <1|2> goto  <cm> [fast|slow]  →  LINAK,<id>,GOTO,<cm>[,FAST|SLOW]
  <1|2> home                    →  LINAK,<id>,HOME   (retract to the end stop)
  <1|2> down  <cm> [fast|slow]  →  LINAK,<id>,DOWN,<cm>[,FAST|SLOW]
  <1|2> up    <cm> [fast|slow]  →  LINAK,<id>,UP,<cm>[,FAST|SLOW]
  <1|2> stop                    →  LINAK,<id>,STOP
  q                             →  quit

Everything is in CENTIMETRES, never seconds.

`goto` is an absolute position measured from home: `1 goto 12` ends at 12 cm
whether it was at 2 cm or 25 cm. `down`/`up` are relative jogs from wherever the
arm is now — handy for nudging, but `goto` is what the FSM uses and what you
want when you care where it ends up. Targets are clamped to the 30 cm stroke.

Omitting the speed tag gets SLOW (1.09 cm/s), the driver's default; FAST is
2.18 cm/s. Actuator 1 is the auger, 2 is the chute.

Topics
──────
  Publishes  /linak_cmd    (std_msgs/String)
  Subscribes /linak_status (std_msgs/String)
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_USAGE = (
    "Commands:  <1|2> goto <cm> [fast|slow]  |  <1|2> home  |"
    "  <1|2> down|up <cm> [fast|slow]  |  <1|2> stop  |  q"
)


# ── Test node ─────────────────────────────────────────────────────────────────

class LinakTestNode(Node):

    def __init__(self) -> None:
        super().__init__("linak_cmd_test_node")
        self._pub = self.create_publisher(String, "/linak_cmd", 10)
        self.create_subscription(String, "/linak_status", self._on_status, 10)
        self.get_logger().info("linak_cmd_test_node ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_status(self, msg: String) -> None:
        print(f"  <- /linak_status: {msg.data}")

    # ── Publish helpers ───────────────────────────────────────────────────

    def send(self, cmd: str) -> None:
        self._pub.publish(String(data=cmd))
        print(f"  -> /linak_cmd: {cmd}")


# ── Interactive input loop ────────────────────────────────────────────────────

def _input_loop(node: LinakTestNode) -> None:
    print(_USAGE)
    while rclpy.ok():
        try:
            raw = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if raw in ("q", "quit", "exit"):
            break

        parts = raw.split()

        if not parts:
            continue

        if parts[0] not in ("1", "2"):
            print("  actuator id must be 1 or 2")
            continue

        actuator = parts[0]

        if len(parts) == 2 and parts[1] == "stop":
            node.send(f"LINAK,{actuator},STOP")

        elif len(parts) == 2 and parts[1] in ("home", "in_max"):
            node.send(f"LINAK,{actuator},HOME")

        elif len(parts) in (3, 4) and parts[1] in ("goto", "down", "up"):
            try:
                value_cm = float(parts[2])
            except ValueError:
                print("  value must be a number (centimetres)")
                continue
            if value_cm < 0:
                print("  value must be non-negative")
                continue

            cmd = f"LINAK,{actuator},{parts[1].upper()},{value_cm}"
            if len(parts) == 4:
                if parts[3] not in ("fast", "slow"):
                    print("  speed must be 'fast' or 'slow'")
                    continue
                cmd += f",{parts[3].upper()}"
            node.send(cmd)

        else:
            print(f"  {_USAGE}")

    rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinakTestNode()
    threading.Thread(
        target=_input_loop, args=(node,), daemon=True, name="linak_test_input"
    ).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
