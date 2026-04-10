import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from manipulation_pkg.planner import Planner
from manipulation_pkg.executor import PickAndPlaceExecutor


class ManipulationPlannerNode(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        self.planner        = Planner(self)
        self.pick_and_place = PickAndPlaceExecutor(self.planner)

        self.should_run = False
        self.create_service(Trigger, '/planner/trigger', self.trigger_cb)

        self.get_logger().info('All ready. Call /planner/trigger to start.')

    def trigger_cb(self, request, response):
        if self.should_run:
            response.success = False
            response.message = 'Already running'
        else:
            self.should_run = True
            response.success = True
            response.message = 'Sequence triggered'
        return response

    def run(self):
        while rclpy.ok():
            if self.should_run:
                self.should_run = False
                self.pick_and_place.run()
            else:
                time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationPlannerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    run_thread = threading.Thread(target=node.run, daemon=True)
    run_thread.start()

    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()