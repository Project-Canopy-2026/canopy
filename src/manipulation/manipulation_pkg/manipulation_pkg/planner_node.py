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

        # lift: position only — orientation is set in run_pick_and_place() once
        # grasp_pose is known (so Pilz LIN step 5 gets the correct approach direction)
        self.lift_pose = self.planner.make_pose(
            cfg.GRASP_POSE['x'],
            cfg.GRASP_POSE['y'],
            cfg.LIFT_POSE['z']
        )

        # ── simulation mode ────────────────────────────────────────────
        # sim_mode=true: no real hardware, no perception, no FSM signals needed.
        # Configurable fake pot position (defaults to GRASP_POSE x/y/z).
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('sim_pot_x', cfg.GRASP_POSE['x'])
        self.declare_parameter('sim_pot_y', cfg.GRASP_POSE['y'])
        self.declare_parameter('sim_pot_z', cfg.GRASP_POSE['z'])
        # ee_link: real robot has 'tool_tcp' (custom gripper TCP);
        # fake/sim URDF only has the stock 'link_eef'
        self.declare_parameter('ee_link', 'tool_tcp')

        self.sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value
        self.planner.ee_link = self.get_parameter('ee_link').get_parameter_value().string_value
        self.logger.info(f'Using end-effector link: {self.planner.ee_link}')

        if self.sim_mode:
            self.logger.info('*** SIMULATION MODE ENABLED ***')
            # Pre-populate pot_center so get_grasp_pose() has data immediately
            fake_pot = PointStamped()
            fake_pot.header.frame_id = cfg.BASE_FRAME
            fake_pot.point.x = self.get_parameter('sim_pot_x').get_parameter_value().double_value
            fake_pot.point.y = self.get_parameter('sim_pot_y').get_parameter_value().double_value
            fake_pot.point.z = self.get_parameter('sim_pot_z').get_parameter_value().double_value
            self.pot_center = fake_pot
            self.logger.info(
                f'[SIM] Fake pot: x={fake_pot.point.x:.3f} '
                f'y={fake_pot.point.y:.3f} z={fake_pot.point.z:.3f}'
            )
            # Auto-trigger the sequence after a short settling delay
            self._sim_timer_handle = self.create_timer(3.0, self._sim_auto_trigger)

        # publishers
        self.call_detection_pub = self.create_publisher(Bool, 'behavior/enable_pot_detection', 10)
        self.seedling_dropped_pub = self.create_publisher(Bool, 'behavior/seedling_dropped', 10)

        # subscribers
        self.create_subscription(PointStamped, 'pot/center_point', self.pot_center_callback, 10)
        self.create_subscription(Bool, 'behavior/do_planting', self.start_planting_callback, 10)
        self.create_subscription(Bool, '/chute_in_position', self.chute_in_position_callback, 10)

        self.should_run = False
        if not self.sim_mode:
            self.pot_center = None
        self.chute_in_position = False


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
        """One-shot timer: auto-start pick-and-place in simulation."""
        self.destroy_timer(self._sim_timer_handle)
        self.logger.info('[SIM] Auto-triggering pick and place')
        self.should_run = True

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

        # Step 1: joint-space to pre-grasp (Pilz PTP for smooth motion; OMPL fallback)
        self.logger.info('Step 1: Pre-grasp (Pilz PTP)...')
        success = self.planner.move_joints(cfg.PRE_GRASP_JOINTS_DEG)
        if not success:
            self.logger.warn('Pilz PTP failed, falling back to OMPL RRTConnect...')
            success = self.planner.move_joints(
                cfg.PRE_GRASP_JOINTS_DEG, pipeline='ompl', planner='RRTConnect')
        if not success:
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

        # update grasp pose
        self.grasp_pose = self.planner.get_grasp_pose(self.pot_center)
        if self.grasp_pose is None:
            self.logger.error('Failed to compute grasp pose (TF lookup failed)')
            return False

        # Propagate the dynamic grasp orientation to lift_pose so that the
        # Pilz LIN lift (step 5) uses the correct approach direction.
        self.lift_pose.orientation = self.grasp_pose.orientation

        self.logger.info(f'[DEBUG] Grasp pose: {self.grasp_pose}')
        self.logger.info(f'[DEBUG] Lift pose:  {self.lift_pose}')

        # Step 3: approach to grasp
        # Solve IK for the grasp pose seeded from the current state (at pre-grasp).
        # Pre-grasp is designed to be close to the grasp in joint space, so the IK
        # solution is nearby — each joint barely moves, giving a clean direct approach.
        # Pilz PTP to those joints is smooth and predictable; no IK re-solving mid-path.
        self.logger.info('Step 3: Grasp approach (IK-seeded Pilz PTP)...')
        grasp_joints_deg = self.planner.grasp_pose_to_joint_values(self.grasp_pose)
        if grasp_joints_deg is not None:
            self.logger.info(f'[DEBUG] Grasp joint target (deg): {[f"{j:.1f}" for j in grasp_joints_deg]}')
            success = self.planner.move_joints(grasp_joints_deg)
        else:
            self.logger.warn('IK for grasp pose failed — falling back to Pilz LIN...')
            success = False

        if not success:
            self.logger.warn('Pilz PTP grasp failed — falling back to Pilz LIN...')
            success = self.planner.move_cartesian(
                self.grasp_pose,
                pipeline='pilz_industrial_motion_planner',
                planner='LIN',
            )
        if not success:
            self.logger.error('Failed at step 3')
            return False

        # Log the actual EEF pose after OMPL grasp move — compare its quaternion
        # to the grasp_pose quaternion to spot IK branch mismatches before Pilz LIN.
        self.planner.log_current_eef_pose('After step 3 (grasp)')

        #Step 4: close gripper
        self.logger.info('Step 4: Close gripper...')
        if not self.planner.close_gripper():
            self.logger.error('Failed at step 4')
            return False

        # Step 5: lift
        # Solve IK for the lift pose seeded from the CURRENT joint state (at grasp).
        # Because the lift is only a small vertical move, the IK solver finds the
        # solution closest to the grasp configuration: joint 2 changes, others stay.
        # Pilz PTP then interpolates those joint angles smoothly — clean, predictable.
        self.logger.info('Step 5: Lift (IK-seeded Pilz PTP)...')
        lift_joints_deg = self.planner.grasp_pose_to_joint_values(self.lift_pose)
        if lift_joints_deg is not None:
            self.logger.info(f'[DEBUG] Lift joint target (deg): {[f"{j:.1f}" for j in lift_joints_deg]}')
            success = self.planner.move_joints(lift_joints_deg)
        else:
            self.logger.warn('IK for lift pose failed — falling back to Pilz LIN...')
            success = False

        if not success:
            self.logger.warn('Pilz PTP lift failed — falling back to Pilz LIN...')
            success = self.planner.move_cartesian(
                self.lift_pose,
                pipeline='pilz_industrial_motion_planner',
                planner='LIN',
            )
        if not success:
            self.logger.error('Failed at step 5')
            return False

        # Step 6a: upright Cartesian transport.
        # wp1 (raise):     grasp orientation  — locked upright
        # wp2/wp3 (swing/lower): DROP_RPY orientation — upright for the drop zone
        # compute_cartesian_path SLERPs between them, so EEF rotates gradually
        # rather than suddenly. No Pilz PTP in this segment.
        self.logger.info('Step 6a: Upright transport (Cartesian path)...')
        transport_waypoints = self.planner.make_transport_waypoints(self.grasp_pose.orientation)
        if not self.planner.move_upright_transport(transport_waypoints):
            self.logger.error('Failed at step 6a — calibrate DROP_APPROACH_POSE in arm_config.py')
            return False

        # Step 6b: Pilz PTP to final drop joint configuration.
        # Covers whatever wasn't reached by 6a (full swing + lower, or just the drop itself).
        # Orientation changes are acceptable once the arm is above the drop zone.
        self.logger.info('Step 6b: Final drop positioning (Pilz PTP)...')
        success = self.planner.move_joints(cfg.DROP_JOINTS_DEG)
        if not success:
            self.logger.warn('Pilz PTP failed, falling back to OMPL RRTConnect...')
            success = self.planner.move_joints(
                cfg.DROP_JOINTS_DEG, pipeline='ompl', planner='RRTConnect')
        if not success:
            self.logger.error('Failed at step 6b')
            return False

        # wait for chute in position command before releasing seedling
        if self.sim_mode:
            self.logger.info('[SIM] Skipping chute_in_position wait — auto-proceeding to drop')
            self.chute_in_position = True
        else:
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