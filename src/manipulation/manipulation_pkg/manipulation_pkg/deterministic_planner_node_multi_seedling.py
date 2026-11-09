import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
#from std_srvs.srv import Trigger

from manipulation_pkg import arm_config as cfg
from manipulation_pkg.planner_actions import Planner

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Empty

class ManipulationPlannerNode(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        self.planner = Planner(self)
        self.logger = self.planner.logger

        # build poses from config
        # self.grasp_pose = self.planner.make_pose_from_dict(cfg.GRASP_POSE)
        #self.grasp_pose = None

        # lift: same X,Y,orientation as grasp, only Z changes
        # self.lift_pose = self.planner.make_pose(
        #     cfg.GRASP_POSE['x'],
        #     cfg.GRASP_POSE['y'],
        #     cfg.LIFT_POSE['z']

        #self.lift_pose.orientation = self.grasp_pose.orientation

        #self.create_service(Trigger, '/planner/trigger', self.trigger_cb)

        #self.get_logger().info('All ready. Call /planner/trigger to start.')

        # seedling 1
        self.pre_grasp_joints_1 = self.planner.deg_to_rad([-87.2, 30.7, -12.7, 24.4, -189.2, 95.5, -87.8])
        self.grasp_joints_1 = self.planner.deg_to_rad([-87.2, 30.7, -11.9, 31, -189.2, 89.2, -87.8])
        self.post_grasp_joints_1 = self.planner.deg_to_rad([-87.1, 5.8, -11.9, 24.1, -189.2, 75, -87.8])
                
        # seedling 2
        self.pre_grasp_joints_2 = self.planner.deg_to_rad([-87.8, 28.8, -10.6, 30.2, -188.8, 88.2, -87.8])
        self.grasp_joints_2 = self.planner.deg_to_rad([-87.9, 32.4, -10, 39.5, -188.8, 80.6, -87.8])
        self.post_grasp_joints_2 = self.planner.deg_to_rad([-87.9, 17.9, -9.5, 36.7, -188.9, 71.1, -87.8])

        # seedling 3
        self.pre_grasp_joints_3 = self.planner.deg_to_rad([-88.3, 34.2, -9.1, 43.3, -188.9, 78.9, -87.8])
        self.grasp_joints_3 = self.planner.deg_to_rad([-88.3, 36, -6.9, 49.6, -184.2, 75.6, -87.1])
        self.post_grasp_joints_3 = self.planner.deg_to_rad([-88.4, 18.8, -7, 46.7, -184.2, 63, -87.1])

        # seedling 4
        self.pre_grasp_joints_4 = self.planner.deg_to_rad([-88.3, 37.9, -7, 52.2, -184.5, 77.3, -87.1])
        self.grasp_joints_4 = self.planner.deg_to_rad([-88.5, 40.1, -6.3, 61.4, -184.5, 65.5, -87.1])
        self.grasp_joints_4 = self.planner.deg_to_rad([-88.5, 40.1, -6.3, 61.4, -184.5, 65.5, -87.1])

        # seedling 5
        self.pre_grasp_joints_5 = self.planner.deg_to_rad([-89.3, 40.3, -6.4, 62.7, -185.7, 66.4, -87.1])
        self.grasp_joints_5 = self.planner.deg_to_rad([-89.4, 45.2, -6.1, 73.2, -185.7, 58.7, -87.1])
        self.post_grasp_joints_5 = self.planner.deg_to_rad([-89.3, 33.1, -5, 72.9, -185.7, 45.4, -87.1])

        # shared transit route from the per-seedling lift to the chute, taken from
        # deterministic_planner_node_0413.py — these route around the planting_assembly
        # collision box, which a direct lift -> drop jump cuts straight through
        self.lift_joints = self.planner.deg_to_rad([-17.4, 0.2, -71.6, 121.7, -184.2, -36.3, -85.6])
        self.intermediate_joints = self.planner.deg_to_rad([-41.3, 26.4, 78.6, 156.6, -223.2, -51, -87.5])
        self.above_chute_joints = self.planner.deg_to_rad([19.7, 41.6, 111.3, 137.2, -233.3, -43.3, -95.6])
        self.drop_joints        = self.planner.deg_to_rad([-24.3, 43.3, 117.7, 86.4, -224.6, -13.4, -85.6])

        # picked in order, one per /behavior/do_planting command
        self.seedling_joints = [
            (self.pre_grasp_joints_1, self.grasp_joints_1, self.post_grasp_joints_1),
            (self.pre_grasp_joints_2, self.grasp_joints_2, self.post_grasp_joints_2),
            (self.pre_grasp_joints_3, self.grasp_joints_3, self.post_grasp_joints_3),
            (self.pre_grasp_joints_4, self.grasp_joints_4, self.post_grasp_joints_4),
            (self.pre_grasp_joints_5, self.grasp_joints_5, self.post_grasp_joints_5),
        ]

        self.seedlings_completed = 0

        # sim_mode: auto-trigger pick-and-place and bypass chute_in_position wait
        self.declare_parameter('sim_mode', False)
        self.sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value
        if self.sim_mode:
            self.logger.info('*** SIMULATION MODE ENABLED ***')

        # publishers
        #self.call_detection_pub = self.create_publisher(Bool, 'behavior/enable_pot_detection', 10)
        self.seedling_dropped_pub = self.create_publisher(Bool, '/behavior/seedling_dropped', 10)
        
        # subscribers
        #self.create_subscription(PointStamped, 'pot/center_point', self.pot_center_callback, 10)
        self.create_subscription(Empty, '/behavior/do_planting', self.start_planting_callback, 10)
        self.create_subscription(Bool, '/chute_in_position', self.chute_in_position_callback, 10)

        self.should_run = False
        self._is_running = False
        #self.pot_center = None
        self.chute_in_position = False

        # In sim_mode, auto-trigger each seedling in turn as the previous one finishes
        if self.sim_mode:
            self._sim_timer_handle = self.create_timer(3.0, self._sim_auto_trigger)


    # def trigger_cb(self, request, response):
    #     if self.should_run:
    #         response.success = False
    #         response.message = 'Already running'
    #     else:
    #         self.should_run = True
    #         response.success = True
    #         response.message = 'Sequence triggered'
    #     return response

    def _sim_auto_trigger(self):
        # only ever runs when sim_mode is true; stands in for the FSM's /behavior/do_planting
        # commands, one per seedling, fired once the previous pick and place has finished
        if self.should_run or self._is_running:
            return

        if self.seedlings_completed >= len(self.seedling_joints):
            self.logger.info(f'[SIM] All {len(self.seedling_joints)} seedlings planted, stopping auto-trigger')
            self._sim_timer_handle.cancel()
            return

        self.logger.info(f'[SIM] Auto-triggering seedling {self.seedlings_completed + 1}/{len(self.seedling_joints)}')
        self.should_run = True
        self.chute_in_position = False


    def start_planting_callback(self, msg: Empty):
        self.logger.info('Received planting command')
        self.logger.info('Received planting command')
        self.logger.info('Received planting command')
        self.logger.info('Received planting command')
        self.logger.info('Received planting command')
        self.logger.info('Received planting command')
        self.logger.info('Received planting command')
        self.should_run = True
        self.chute_in_position = False


    def chute_in_position_callback(self, msg: Bool):
        if msg.data:
            self.logger.info('Received chute in position command')
            self.chute_in_position = True
        else:
            self.logger.info('Received chute not in position command')
            self.chute_in_position = False


    def pot_center_callback(self, msg: PointStamped):
        self.pot_center = msg


    def run_pick_and_place(self):
        idx = self.seedlings_completed
        if idx >= len(self.seedling_joints):
            self.logger.warn(f'All {len(self.seedling_joints)} seedlings already planted, ignoring command')
            return False

        pre_grasp_joints, grasp_joints, post_grasp_joints = self.seedling_joints[idx]
        self.logger.info(f'=== Starting pick and place for seedling {idx + 1}/{len(self.seedling_joints)} ===')

        # Step 0: set up collision scene first
        self.planner.setup_collision_scene()

        # Step 1: joint-space to pre-grasp
        self.get_logger().info('Step 1: Moving to Pre Grasp')
        if not self.planner._call_plan_joint(pre_grasp_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        # Step 2: open gripper
        self.logger.info('Step 2: Open gripper...')
        if not self.planner.open_gripper():
            self.logger.error('Failed at step 2')
            return False

        # move to grasp
        self.get_logger().info('Step 3: Moving towards to grasp...')
        if not self.planner._call_plan_joint(grasp_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        #Step 4: close gripper
        self.logger.info('Step 4: Close gripper...')
        if not self.planner.close_gripper():
            self.logger.error('Failed at step 4')
            return False

        # Step 5: lift
        self.get_logger().info('Step 5: Lifting...')
        if not self.planner._call_plan_joint(post_grasp_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        # Step 6: transit around the planting assembly to above the chute
        self.get_logger().info('Step 6: Transiting to above chute')
        for name, joints in (('transit', self.lift_joints),
                             ('intermediate', self.intermediate_joints),
                             ('above chute', self.above_chute_joints)):
            self.get_logger().info(f'Step 6: Moving to {name}')
            if not self.planner._call_plan_joint(joints):
                return False
            if not self.planner._call_plan_exec():
                return False

        # wait above the chute, not at the drop pose, so the arm stays clear
        # of the chute's space until it reports itself in position
        if self.sim_mode:
            self.logger.info('[SIM] Skipping chute_in_position wait — auto-proceeding to drop')
            self.chute_in_position = True
        else:
            self.logger.info(f"chute_in_position: {self.chute_in_position}")
            while not self.chute_in_position:
                self.logger.info('Waiting for chute to be in position before dropping seedling...')
                time.sleep(0.5)

        # Step 6b: descend into the chute
        self.get_logger().info('Step 6b: Moving to drop the seedling into chute')
        if not self.planner._call_plan_joint(self.drop_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        # Step 7: release
        self.logger.info('Step 7: Releasing seedling...')
        if self.chute_in_position:
            self.logger.info('Chute is in position, proceeding to drop')

            #open gripper to release seedling
            if not self.planner.open_gripper():
                self.logger.error('Failed to open gripper at drop pose')
                return
            
            self.logger.info('Seedling dropped successfully')
            self.seedling_dropped_pub.publish(Bool(data=True))

            # only advance on a successful drop, so a failed attempt retries the same seedling
            self.seedlings_completed += 1
        else:
            self.logger.info('Received chute not in position command, aborting drop')

        # step 8: move back to pregrasp
        self.get_logger().info('Step 8: Moving back to pregrasp joints')
        if not self.planner._call_plan_joint(self.above_chute_joints):
            return False
        if not self.planner._call_plan_exec():
            return False
        if not self.planner._call_plan_joint(self.intermediate_joints):
            return False
        if not self.planner._call_plan_exec():
            return False
        if not self.planner._call_plan_joint(self.lift_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        self.get_logger().info('Step 8: Moving to Pre Grasp')
        if not self.planner._call_plan_joint(pre_grasp_joints):
            return False
        if not self.planner._call_plan_exec():
            return False


        self.logger.info(f'=== Pick and place complete ({self.seedlings_completed}/{len(self.seedling_joints)} seedlings planted) ===')
        return True

    def run(self):
        while rclpy.ok():
            if self.should_run:
                self.should_run = False
                self._is_running = True
                try:
                    self.run_pick_and_place()
                finally:
                    self._is_running = False
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
