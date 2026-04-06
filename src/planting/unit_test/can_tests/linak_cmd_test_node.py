#!/usr/bin/env python3
# ============================================================
#  LINAK Test Node — quick interactive tester for linak_can_node
#
#  Run alongside linak_can_node:
#    ros2 run planting_controller linak_test
#
#  Commands:
#    1 down 8   → LINAK,1,DOWN,8.0
#    2 up 6     → LINAK,2,UP,6.0
#    1 out_max  → LINAK,1,OUT_MAX
#    1 in_max   → LINAK,1,IN_MAX
#    1 stop     → LINAK,1,STOP
#    q          → quit
# ============================================================

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LinakTestNode(Node):

    def __init__(self):
        super().__init__('linak_test')
        self.pub = self.create_publisher(String, '/linak_cmd', 10)
        self.create_subscription(String, '/linak_status', self._on_status, 10)

    def _on_status(self, msg: String):
        print(f'  ← {msg.data}')

    def send(self, cmd: str):
        self.pub.publish(String(data=cmd))
        print(f'  → {cmd}')


def input_loop(node: LinakTestNode):
    print("LINAK tester ready. Commands:  <1|2> <down|up> <secs>  |  <1|2> <out_max|in_max>  |  <1|2> stop  |  q")
    while rclpy.ok():
        try:
            raw = input('> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if raw in ('q', 'quit'):
            break

        parts = raw.split()

        if not parts or parts[0] not in ('1', '2'):
            print('actuator must be 1 or 2')
            continue

        actuator = parts[0]

        if len(parts) == 2 and parts[1] == 'stop':
            node.send(f'LINAK,{actuator},STOP')

        elif len(parts) == 2 and parts[1] == 'out_max':
            node.send(f'LINAK,{actuator},OUT_MAX')

        elif len(parts) == 2 and parts[1] == 'in_max':
            node.send(f'LINAK,{actuator},IN_MAX')

        elif len(parts) == 3 and parts[1] in ('down', 'up'):
            try:
                duration = float(parts[2])
            except ValueError:
                print('  duration must be a number (seconds)')
                continue
            node.send(f'LINAK,{actuator},{parts[1].upper()},{duration}')

        else:
            print('  usage:  <1|2> <down|up> <secs>  |  <1|2> <out_max|in_max>  |  <1|2> stop')

    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LinakTestNode()
    threading.Thread(target=input_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()