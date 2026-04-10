import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
#from std_srvs.srv import Trigger

from manipulation_pkg import arm_config as cfg
from manipulation_pkg.planner_actions import Planner

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

class ManipulationPlannerNode(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        self.planner = Planner(self)
        self.logger = self.planner.logger

        # build poses from config
        #self.grasp_pose = self.planner.make_pose_from_dict(cfg.GRASP_POSE)
        self.grasp_pose = None

        # lift: same X,Y,orientation as grasp, only Z changes
        self.lift_pose = self.planner.make_pose(
            cfg.GRASP_POSE['x'],
            cfg.GRASP_POSE['y'],
            cfg.LIFT_POSE['z']
        )
        #elf.lift_pose.orientation = self.grasp_pose.orientation

        #self.create_service(Trigger, '/planner/trigger', self.trigger_cb)

        #self.get_logger().info('All ready. Call /planner/trigger to start.')

        # publishers
        self.call_detection_pub = self.create_publisher(Bool, 'behavior/enable_pot_detection', 10)
        self.seedling_dropped_pub = self.create_publisher(Bool, 'behavior/seedling_dropped', 10)
        
        # subscribers
        #self.create_subscription(PointStamped, 'pot/center_point', self.pot_center_callback, 10)
        self.create_subscription(Bool, 'behavior/do_planting', self.start_planting_callback, 10)
        self.create_subscription(Bool, '/chute_in_position', self.chute_in_position_callback, 10)

        self.should_run = False
        self.pot_center = None
        self.chute_in_position= False


    # def trigger_cb(self, request, response):
    #     if self.should_run:
    #         response.success = False
    #         response.message = 'Already running'
    #     else:
    #         self.should_run = True
    #         response.success = True
    #         response.message = 'Sequence triggered'
    #     return response

    def start_planting_callback(self, msg: Bool):
        if msg.data:
            self.logger.info('Received planting command')
            self.should_run = True
        else:
            self.logger.info('Received stop command')
            self.should_run = False


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
        self.logger.info('Step 1: Pre-grasp (joint)...')
        if not self.planner.move_joints(cfg.PRE_GRASP_JOINTS_DEG):
            self.logger.error('Failed at step 1')
            return False

        # Step 2: open gripper
        # self.logger.info('Step 2: Open gripper...')
        # if not self.planner.open_gripper():
        #     self.logger.error('Failed at step 2')
        #     return False

        # run pot detector until we get a valid detection
        # while self.pot_center is None:
        #     self.logger.info('Running pot detection')
        #     self.call_detection_pub.publish(Bool(data=True))
        #     time.sleep(0.5)

        # self.logger.info(f'Pot detected at: {self.pot_center}')
        # # pot center in link_base (arm base) frame

        # # update grasp poseshould_run
        # self.grasp_pose = self.planner.get_grasp_pose(self.pot_center)

        # self.logger.info(f'Grasp pose calculated: {self.grasp_pose}')

        # # get joint values of grasp pose
        # grasp_joints = self.planner.grasp_pose_to_joint_values(self.grasp_pose)

        # if grasp_joints is None or len(grasp_joints) < 7:
        #     self.logger.error(f'IK returned invalid joints: {grasp_joints}')
        #     return False

        # planner move to grasp
        self.logger.info('Step 3: Grasp Move (Pilz LIN)...')
        success = self.planner.move_cartesian(
            self.grasp_pose,
            pipeline='pilz_industrial_motion_planner',
            planner='LIN'
        )
        if not success:
            self.logger.warn('Pilz LIN failed, falling back to OMPL...')
            success = self.planner.move_cartesian(self.grasp_pose)
        if not success:
            self.logger.error('Failed at step 3')
            return False

        #Step 4: close gripper
        self.logger.info('Step 4: Close gripper...')
        if not self.planner.close_gripper():
            self.logger.error('Failed at step 4')
            return False

        # Step 5: lift
        self.logger.info('Step 5: Lift (Pilz LIN)...')
        success = self.planner.move_cartesian(
            self.lift_pose,
            pipeline='pilz_industrial_motion_planner',
            planner='LIN',
            constrained=True,
            reference_pose=self.grasp_pose
        )
        if not success:
            self.logger.warn('Pilz LIN lift failed, falling back to OMPL...')
            success = self.planner.move_cartesian(
                self.lift_pose,
                constrained=True,
                reference_pose=self.grasp_pose
            )
        if not success:
            self.logger.error('Failed at step 5')
            return False

        # Step 6: move to drop
        self.logger.info('Step 6: Drop (joint)...')
        if not self.planner.move_joints(cfg.DROP_JOINTS_DEG):
            self.logger.error('Failed at step 6')
            return False

        # wait for chute in position command before releasing seedling
        while not self.chute_in_position:
            self.logger.info('Waiting for chute to be in position before dropping seedling...')
            time.sleep(0.5)

        # Step 7: release
        if self.chute_in_position:
            self.logger.info('Chute is in position, proceeding to drop')

            # open gripper to release seedling
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