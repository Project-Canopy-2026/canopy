import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from manipulation_pkg import arm_config as cfg
from manipulation_pkg.planner_actions import Planner

from geometry_msgs.msg import PointStamped


class ManipulationPlannerNode(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        self.planner = Planner(self)
        self.logger = self.planner.logger

        # build poses from config
        self.grasp_pose = self.planner.make_pose_from_dict(cfg.GRASP_POSE)

        # lift: same X,Y,orientation as grasp, only Z changes
        self.lift_pose = self.planner.make_pose(
            cfg.GRASP_POSE['x'],
            cfg.GRASP_POSE['y'],
            cfg.LIFT_POSE['z']
        )
        self.lift_pose.orientation = self.grasp_pose.orientation

        self.should_run = False
        self.create_service(Trigger, '/planner/trigger', self.trigger_cb)

        self.get_logger().info('All ready. Call /planner/trigger to start.')

        self.call_detection = self.create_publisher(Bool, 'behavior/enable_pot_detection', 10)
        
        self.create_subscription(PointStamped, 'pot/center_point', self.pot_center_callback(), 10)

        self.pot_center = None


    def trigger_cb(self, request, response):
        if self.should_run:
            response.success = False
            response.message = 'Already running'
        else:
            self.should_run = True
            response.success = True
            response.message = 'Sequence triggered'
        return response


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
        self.logger.info('Step 2: Open gripper...')
        if not self.planner.open_gripper():
            self.logger.error('Failed at step 2')
            return False

        # run pot detector
        # call a few times?
        self.call_detection.publish(True)
        
        # update grasp pose
        self.grasp_pose = self.planner.get_grasp_pose(self.pot_center)

        # Step 3: move to grasp
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

        # Step 4: close gripper
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

        # Step 7: release
        self.logger.info('Step 7: Release...')
        if not self.planner.open_gripper():
            self.logger.error('Failed at step 7')
            return False

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