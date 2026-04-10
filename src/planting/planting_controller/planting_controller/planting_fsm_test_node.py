#!/usr/bin/env python3
# ============================================================
#  Manual tester for planting_fsm_outmax
#
#  Run alongside planting_fsm_outmax:
#    ros2 run planting_controller planting_fsm_outmax &
#    python3 planting_fsm_test_node.py
#
#  Commands:
#    p   → publish /behavior/do_plant    (Empty)  — start planting sequence
#    s   → publish seedling_dropped      (Bool)   — signal seedling dropped
#    q   → quit
# ============================================================

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class PlantingFsmTestNode(Node):

    def __init__(self):
        super().__init__('planting_fsm_test')

        self._do_plant_pub     = self.create_publisher(Empty,  '/behavior/do_plant',  10)
        self._seedling_pub     = self.create_publisher(Bool,   'seedling_dropped',    10)

        self.create_subscription(String, 'planting_state',  self._on_state,         10)
        self.create_subscription(String, '/linak_status',   self._on_linak_status,  10)
        self.create_subscription(String, '/arduino_status', self._on_arduino_status, 10)

    def _on_state(self, msg: String):
        print(f'  [state]   {msg.data}')

    def _on_linak_status(self, msg: String):
        print(f'  [linak]   {msg.data}')

    def _on_arduino_status(self, msg: String):
        print(f'  [arduino] {msg.data}')

    def send_do_plant(self):
        self._do_plant_pub.publish(Empty())
        print('  → /behavior/do_plant  (Empty)')

    def send_seedling_dropped(self):
        self._seedling_pub.publish(Bool(data=True))
        print('  → seedling_dropped  True')


def input_loop(node: PlantingFsmTestNode):
    print('Planting FSM tester ready.')
    print('  p  — do_plant (start sequence)')
    print('  s  — seedling_dropped')
    print('  q  — quit')
    while rclpy.ok():
        try:
            cmd = input('> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'q':
            break
        elif cmd == 'p':
            node.send_do_plant()
        elif cmd == 's':
            node.send_seedling_dropped()
        else:
            print('  unknown command — use p, s, or q')

    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PlantingFsmTestNode()
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
