#!/usr/bin/env python3
"""
linak_cmd_test_node.py — interactive CLI tester for linak_can_node.

Run alongside linak_can_node:
  ros2 run planting_controller linak_can_node &
  python3 linak_cmd_test_node.py

Command syntax
──────────────
  <1|2> down  <secs>   →  LINAK,<id>,DOWN,<secs>
  <1|2> up    <secs>   →  LINAK,<id>,UP,<secs>
  <1|2> stop           →  LINAK,<id>,STOP
  <1|2> clear          →  LINAK,<id>,CLEAR
  q                    →  quit

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
    "Commands:  <1|2> down <secs>  |  <1|2> up <secs>  |"
    "  <1|2> stop  |  <1|2> clear  |  q"
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

        elif len(parts) == 2 and parts[1] == "clear":
            node.send(f"LINAK,{actuator},CLEAR")

        elif len(parts) == 2 and parts[1] == "out_max":
            node.send(f"LINAK,{actuator},OUT_MAX")

        elif len(parts) == 2 and parts[1] == "in_max":
            node.send(f"LINAK,{actuator},IN_MAX")

        elif len(parts) == 3 and parts[1] in ("down", "up"):
            try:
                duration = float(parts[2])
            except ValueError:
                print("  duration must be a number (seconds)")
                continue
            node.send(f"LINAK,{actuator},{parts[1].upper()},{duration}")

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
