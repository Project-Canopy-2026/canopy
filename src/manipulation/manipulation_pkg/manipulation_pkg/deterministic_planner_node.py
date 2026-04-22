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

        self.pre_grasp_joints = self.planner.deg_to_rad([-61.1, -3.7, -30.5, 2.0,  -181.5,  84.7, -91.7])
        self.grasp_joints     = self.planner.deg_to_rad([-27.6, 16.7, -53.4, 21.3, -168.8,  76.8, -78.9])
        self.lift_joints      = self.planner.deg_to_rad([-41.5, -12.8, -46.9, 19.6, -176.6,  62.0, -100.9])
        self.drop_joints      = self.planner.deg_to_rad([-35.0, -25.0, 0.0, 75.0, -160.0,  -10.0, -110.0])

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
        #self.pot_center = None
        self.chute_in_position = False

        # In sim_mode, auto-trigger the sequence once after a short delay
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
        self.logger.info('=== Starting pick and place ===')

        # Step 0: set up collision scene first
        self.planner.setup_collision_scene()

        # Step 1: joint-space to pre-grasp
        self.get_logger().info('Step 1: Moving to Pre Grasp')
        if not self.planner._call_plan_joint(self.pre_grasp_joints):
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
        if not self.planner._call_plan_joint(self.grasp_joints):
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
        if not self.planner._call_plan_joint(self.lift_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        # Step 6: move to drop
        self.get_logger().info('Step 6: Moving to Waypoint B')
        if not self.planner._call_plan_joint(self.drop_joints):
            return False
        if not self.planner._call_plan_exec():
            return False

        # wait for chute in position command before releasing seedling
        if self.sim_mode:
            self.logger.info('[SIM] Skipping chute_in_position wait — auto-proceeding to drop')
            self.chute_in_position = True
        else:
            self.logger.info(f"chute_in_position: {self.chute_in_position}")
            while not self.chute_in_position:
                self.logger.info('Waiting for chute to be in position before dropping seedling...')
                time.sleep(0.5)

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
        else:
            self.logger.info('Received chute not in position command, aborting drop')

        self.logger.info('=== Pick and place complete ===')
        return True

    def run(self):
        while rclpy.ok():
            if self.should_run:
                self.should_run = False
                self.run_pick_and_place()
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