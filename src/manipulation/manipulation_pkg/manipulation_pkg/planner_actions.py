import math
import time
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, Constraints,
    OrientationConstraint, PositionConstraint,
    BoundingVolume, JointConstraint, RobotState,
    DisplayTrajectory
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
import tf2_ros

from xarm_msgs.srv import PlanExec, PlanJoint

from manipulation_pkg import arm_config as cfg


class Planner:
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # action client
        self.move_client = ActionClient(node, MoveGroup, '/move_action')

        # gripper service clients
        self.gripper_open  = node.create_client(Trigger, '/gripper/open')
        self.gripper_close = node.create_client(Trigger, '/gripper/close')

        # planning scene publisher
        self.scene_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)

        # trajectory visualization publisher (RViz Motion Planning plugin)
        self.display_traj_pub = node.create_publisher(
            DisplayTrajectory, '/display_planned_path', 10
        )

        # execute pre-planned trajectory (separate from MoveGroup plan+exec)
        self.exec_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

        self.logger.info('Waiting for MoveGroup action server...')
        self.move_client.wait_for_server()
        self.logger.info('Waiting for ExecuteTrajectory action server...')
        self.exec_client.wait_for_server()
        #self._wait_for_services()
        self.logger.info('Planner ready.')

        self.ee_link = "tool_tcp"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self.cartesian_path_client = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        #self.logger.info('Waiting for /compute_ik service...')
        #self.ik_client.wait_for_service()
        # self.arm_group_name = "xarm7"

        self.plan_joint = node.create_client(PlanJoint, '/xarm_joint_plan')
        self.plan_exec = node.create_client(PlanExec, '/xarm_exec_plan')

    # ── helpers ────────────────────────────────────────────────────────
    def _wait_for_services(self):
        for client in [self.gripper_open, self.gripper_close]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.logger.warn(f'Waiting for {client.srv_name}...')

    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        qx = math.sin(r/2)*math.cos(p/2)*math.cos(y/2) - math.cos(r/2)*math.sin(p/2)*math.sin(y/2)
        qy = math.cos(r/2)*math.sin(p/2)*math.cos(y/2) + math.sin(r/2)*math.cos(p/2)*math.sin(y/2)
        qz = math.cos(r/2)*math.cos(p/2)*math.sin(y/2) - math.sin(r/2)*math.sin(p/2)*math.cos(y/2)
        qw = math.cos(r/2)*math.cos(p/2)*math.cos(y/2) + math.sin(r/2)*math.sin(p/2)*math.sin(y/2)
        return qx, qy, qz, qw

    def make_pose(self, x, y, z, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        qx, qy, qz, qw = self.euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def make_pose_from_dict(self, d):
        return self.make_pose(
            d['x'], d['y'], d['z'],
            d.get('roll', 0.0), d.get('pitch', 0.0), d.get('yaw', 0.0)
        )

    def deg_to_rad(self, degrees_list):
        return [math.radians(float(v)) for v in degrees_list]

    # ── collision scene ────────────────────────────────────────────────
    def setup_collision_scene(self):
        self.logger.info('Setting up collision scene...')

        # Remove any stale objects first
        scene = PlanningScene()
        scene.is_diff = True
        for name in ['rail_left', 'rail_right']:
            obj = CollisionObject()
            obj.id = name
            obj.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)
        time.sleep(0.5)

        scene = PlanningScene()
        scene.is_diff = True

        for name, x, y, z, lx, ly, lz in cfg.COLLISION_BOXES:
            obj = CollisionObject()
            obj.header.frame_id = cfg.BASE_FRAME
            obj.id = name
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [float(lx), float(ly), float(lz)]
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0
            obj.primitives = [box]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obj)

        for _ in range(5):
            self.scene_pub.publish(scene)
            time.sleep(0.2)
        self.logger.info('Collision scene ready.')

    # ── upright constraint ─────────────────────────────────────────────
    def upright_constraint(self, reference_pose):
        oc = OrientationConstraint()
        oc.header.frame_id = cfg.BASE_FRAME
        oc.link_name = cfg.EEF_LINK
        oc.orientation = reference_pose.orientation
        oc.absolute_x_axis_tolerance = cfg.UPRIGHT_TOL_XY
        oc.absolute_y_axis_tolerance = cfg.UPRIGHT_TOL_XY
        oc.absolute_z_axis_tolerance = cfg.UPRIGHT_TOL_Z
        oc.weight = 1.0
        c = Constraints()
        c.name = 'upright'
        c.orientation_constraints.append(oc)
        return c

    # ── planning / visualization / execution ───────────────────────────

    def _plan(self, goal):
        """Request a plan from MoveGroup (no execution). Returns RobotTrajectory or None."""
        goal.planning_options.plan_only = True
        future = self.move_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.01)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.01)
        result = result_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return result.planned_trajectory
        self.logger.warn(
            f'Planning failed (code {result.error_code.val})'
            f' | pipeline={goal.request.pipeline_id} planner={goal.request.planner_id}'
        )
        return None

    def _visualize_trajectory(self, trajectory):
        """Publish planned trajectory to /display_planned_path for RViz inspection."""
        msg = DisplayTrajectory()
        msg.model_id = cfg.PLANNING_GROUP
        msg.trajectory = [trajectory]
        msg.trajectory_start.is_diff = True
        self.display_traj_pub.publish(msg)
        self.logger.info(
            f'[VIZ] Trajectory published to /display_planned_path — '
            f'executing in {cfg.VIZ_PAUSE_SEC:.1f} s'
        )
        time.sleep(cfg.VIZ_PAUSE_SEC)

    def _execute_trajectory(self, trajectory):
        """Execute a pre-planned RobotTrajectory via /execute_trajectory action."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.exec_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.01)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error('ExecuteTrajectory goal rejected')
            return False
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.01)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error(f'Execution failed (code {result.error_code.val})')
            return False
        return True

    def _send_goal(self, goal, retries=None):
        retries = retries or cfg.PLAN_RETRIES
        for attempt in range(retries):
            trajectory = self._plan(goal)
            if trajectory is None:
                self.logger.warn(f'Planning failed, attempt {attempt+1}/{retries}')
                continue

            if cfg.VIZ_BEFORE_EXEC:
                self._visualize_trajectory(trajectory)

            if self._execute_trajectory(trajectory):
                return True

            self.logger.warn(f'Execution failed, attempt {attempt+1}/{retries}')

        self.logger.error('Move failed after all retries')
        return False

    # ── move to joint angles ───────────────────────────────────────────
    def move_joints(self, joint_angles_deg, pipeline='pilz_industrial_motion_planner',
                    planner='PTP', retries=None):
        """Plan and execute a joint-space move.

        Defaults to Pilz PTP, which produces smooth point-to-point motions with
        trapezoidal velocity profiles.  Pass pipeline='ompl', planner='RRTConnect'
        as a fallback when Pilz PTP fails.
        """
        joint_angles_rad = self.deg_to_rad(joint_angles_deg)
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = cfg.PLANNING_GROUP
        req.num_planning_attempts = cfg.NUM_PLANNING_ATTEMPTS
        req.allowed_planning_time = cfg.PLANNING_TIME
        req.max_velocity_scaling_factor = cfg.MAX_VELOCITY_SCALING
        req.max_acceleration_scaling_factor = cfg.MAX_ACCELERATION_SCALING
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.start_state.is_diff = True

        joint_constraints = Constraints()
        for name, angle in zip(cfg.JOINT_NAMES, joint_angles_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.joint_constraints.append(jc)
        req.goal_constraints.append(joint_constraints)

        return self._send_goal(goal, retries)

   
    def move_cartesian(self, pose, pipeline='ompl', planner='RRTConnect',
                       constrained=False, reference_pose=None, retries=None):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = cfg.PLANNING_GROUP
        req.num_planning_attempts = cfg.NUM_PLANNING_ATTEMPTS
        req.allowed_planning_time = cfg.PLANNING_TIME
        req.max_velocity_scaling_factor = cfg.MAX_VELOCITY_SCALING
        req.max_acceleration_scaling_factor = cfg.MAX_ACCELERATION_SCALING
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.start_state.is_diff = True

        # position constraint
        pc = PositionConstraint()
        pc.header.frame_id = cfg.BASE_FRAME
        pc.link_name = cfg.EEF_LINK
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.001]
        bv.primitives = [sp]
        bv.primitive_poses = [pose]
        pc.constraint_region = bv
        pc.weight = 1.0

        # orientation constraint (goal) TODO: Loosen the tolerance to increase motion planning success rate
        oc = OrientationConstraint()
        oc.header.frame_id = cfg.BASE_FRAME
        oc.link_name = cfg.EEF_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01 
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)
        goal_constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(goal_constraints)

        # path constraint (upright during transport)
        if constrained and reference_pose:
            req.path_constraints = self.upright_constraint(reference_pose)

        return self._send_goal(goal, retries)

    # ── gripper ────────────────────────────────────────────────────────
    def open_gripper(self):
        return self._call_gripper(self.gripper_open, 'open')

    def close_gripper(self):
        return self._call_gripper(self.gripper_close, 'close')

    # def _call_gripper(self, client, action):
    #     req = Trigger.Request()
    #     future = client.call_async(req)
    #     while not future.done():
    #         time.sleep(0.01)
    #     result = future.result()
    #     if not result.success:
    #         self.logger.error(f'Gripper {action} failed')
    #         return False
    #     self.logger.info(f'Gripper {action} success')
    #     return True
    
    def _call_gripper(self, client, action):
        req = Trigger.Request()
        future = client.call_async(req)
        while not future.done():
            time.sleep(0.05)
        result = future.result()
        if not result.success:
            self.logger.error(f'Gripper {action} failed')
            return False
        self.logger.info(f'Gripper {action} success')
        return True

    # ── debug helpers ──────────────────────────────────────────────────
    def log_current_eef_pose(self, label=''):
        """Log the current EEF pose via TF — useful for diagnosing IK branch issues."""
        pose = self.get_gripper_pose()
        if pose is None:
            return
        prefix = f'[{label}] ' if label else ''
        self.logger.info(
            f'{prefix}EEF pose — '
            f'pos: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})  '
            f'quat: ({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, '
            f'{pose.orientation.z:.4f}, {pose.orientation.w:.4f})'
        )

    # ── grasp pose ──────────────────────────────────────────────────────
    def get_gripper_pose(self):
        # gives gripper pose relative to arm base frame (link_base)
        try:
            transform = self.tf_buffer.lookup_transform(
                cfg.BASE_FRAME, self.ee_link, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.logger.error(f'TF lookup failed: {e}')
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose


    def normalize(self, v, eps=1e-8):
        norm = np.linalg.norm(v)
        if norm < eps:
            raise ValueError("Cannot normalize near-zero vector")
        return v / norm


    def get_grasp_pose(self, pot_center):
        # gives grasp pose realtive to arm base frame (link_base)
        # pot_center is pointStamp type

        # get gripper cartesian position
        gripper_pose = self.get_gripper_pose()
        if gripper_pose is None:
            self.logger.error('Cannot compute grasp pose: TF lookup failed')
            return None

        self.logger.info(f'Gripper pose at grasp calculation: {gripper_pose}')

        gripper_pos = np.array([
            gripper_pose.position.x,
            gripper_pose.position.y,
            gripper_pose.position.z
        ], dtype=float)

        # get gripper current orientation as rotation matrix
        current_R = Rot.from_quat([
            gripper_pose.orientation.x,
            gripper_pose.orientation.y,
            gripper_pose.orientation.z,
            gripper_pose.orientation.w
        ]).as_matrix()

        y_ref = current_R[:, 1]  # second column = current Y axis in base frame

        pot_pos = np.array([
            pot_center.point.x,
            pot_center.point.y,
            pot_center.point.z
        ], dtype=float)

        # calculate vector from pot to gripper
        grasp_vector = pot_pos - gripper_pos

        # approaching axis
        z_gripper_axis = self.normalize(grasp_vector)

        # sideways axis
        x_gripper_axis = np.cross(y_ref, z_gripper_axis)

        # if ref parallel to z axis
        if np.linalg.norm(x_gripper_axis) < 1e-8:
            x_gripper_axis = np.cross(current_R[:, 0], z_gripper_axis)

        x_gripper_axis = self.normalize(x_gripper_axis)

        # remaining axis
        y_gripper_axis = self.normalize(np.cross(z_gripper_axis, x_gripper_axis))
        
        # renormalize x to make sure orthogonal
        x_gripper_axis = self.normalize(np.cross(y_gripper_axis, z_gripper_axis))

         # rotation matrix
        R = np.column_stack((x_gripper_axis, y_gripper_axis, z_gripper_axis))

        # convert to quaternion [x,y,z,w]
        orientation = Rot.from_matrix(R).as_quat()

        # output needs to be same as make_pose output (x,y,z position & quaternion)
        grasp_pose = Pose()
        grasp_pose.position.x = float(pot_center.point.x)
        grasp_pose.position.y = float(pot_center.point.y)
        grasp_pose.position.z = float(pot_center.point.z)
        grasp_pose.orientation.x = orientation[0]
        grasp_pose.orientation.y = orientation[1]
        grasp_pose.orientation.z = orientation[2]
        grasp_pose.orientation.w = orientation[3]

        return grasp_pose

    
    def grasp_pose_to_joint_values(self, grasp_pose):
        """Solve IK for `grasp_pose` seeded from the current robot state.

        Using is_diff=True means the solver uses the live joint state as its seed,
        so it finds the IK solution nearest to the current configuration rather than
        exploring other branches. For a small lift this means only joint 2 moves.

        Returns joint angles in degrees (matching move_joints input), or None on failure.
        """
        self.ik_client.wait_for_service()
        req = GetPositionIK.Request()
        req.ik_request.group_name = cfg.PLANNING_GROUP
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.is_diff = True
        req.ik_request.timeout.sec = 1

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = cfg.BASE_FRAME
        pose_stamped.pose = grasp_pose
        req.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(req)
        while not future.done():
            time.sleep(0.01)

        result = future.result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.warn(f'IK failed with error code {result.error_code.val}')
            return None

        # IK returns positions for ALL robot joints in arbitrary order.
        # Map by name and extract only the 7 arm joints in cfg.JOINT_NAMES order,
        # then convert rad → deg to match move_joints()'s expected input units.
        state = result.solution.joint_state
        name_to_pos = dict(zip(state.name, state.position))
        try:
            joint_angles_rad = [name_to_pos[name] for name in cfg.JOINT_NAMES]
        except KeyError as e:
            self.logger.warn(f'IK solution missing joint {e}')
            return None
        return [math.degrees(a) for a in joint_angles_rad]


    # ── upright transport ──────────────────────────────────────────────
    def _slerp_orientation(self, q1, q2, t):
        """Spherical linear interpolation between two geometry_msgs Quaternions at parameter t."""
        from scipy.spatial.transform import Rotation as R, Slerp
        import numpy as np
        rots = R.from_quat([
            [q1.x, q1.y, q1.z, q1.w],
            [q2.x, q2.y, q2.z, q2.w],
        ])
        slerp = Slerp([0.0, 1.0], rots)
        mid = slerp(t).as_quat()
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x, q.y, q.z, q.w = float(mid[0]), float(mid[1]), float(mid[2]), float(mid[3])
        return q

    def make_transport_waypoints(self, grasp_orientation):
        """Build via-poses for upright Cartesian transport with region-appropriate orientations.

        wp1 (raise):        grasp_orientation — dynamic, confirmed reachable at lift height.
        wp2 (mid-swing):    SLERP(grasp, drop, 0.5) — explicit halfway orientation that gives
                            the IK solver a stable seed and prevents wrist flips across the
                            large yaw change from grasp (yaw≈0°) to drop (yaw≈85°).
        wp3 (above drop):   DROP_APPROACH orientation — reachable in the drop zone region.
        wp4 (lower):        DROP_APPROACH orientation — same as wp3.

        compute_cartesian_path SLERPs between consecutive waypoints; with an explicit
        mid-swing point the per-segment yaw change is ~42° instead of ~85°, keeping
        the IK solver in the same solution branch throughout.

        Path:
          current position (at grasp/lift height)
            → raise to TRANSPORT_Z                  [grasp orientation]
            → mid-swing (midpoint x/y, TRANSPORT_Z) [SLERP 50%]
            → above DROP_APPROACH (TRANSPORT_Z)      [drop orientation]
            → lower to DROP_APPROACH                 [drop orientation]
        """
        drop_orientation = self.make_pose_from_dict(cfg.DROP_APPROACH_POSE).orientation
        mid_orientation  = self._slerp_orientation(grasp_orientation, drop_orientation, 0.5)

        # Midpoint position between lift and drop approach (horizontal)
        mid_x = (cfg.LIFT_POSE['x'] + cfg.DROP_APPROACH_POSE['x']) / 2.0
        mid_y = (cfg.LIFT_POSE['y'] + cfg.DROP_APPROACH_POSE['y']) / 2.0

        def _pose(x, y, z, orientation):
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation = orientation
            return p

        return [
            # 1. Raise straight up — hold exact grasp orientation
            _pose(cfg.LIFT_POSE['x'], cfg.LIFT_POSE['y'], cfg.TRANSPORT_Z,
                  grasp_orientation),
            # 2. Mid-swing — explicit SLERP(50%) orientation prevents wrist flip
            _pose(mid_x, mid_y, cfg.TRANSPORT_Z,
                  mid_orientation),
            # 3. Above drop approach — drop orientation
            _pose(cfg.DROP_APPROACH_POSE['x'], cfg.DROP_APPROACH_POSE['y'], cfg.TRANSPORT_Z,
                  drop_orientation),
            # 4. Lower to drop approach height — hold drop orientation
            _pose(cfg.DROP_APPROACH_POSE['x'], cfg.DROP_APPROACH_POSE['y'],
                  cfg.DROP_APPROACH_POSE['z'], drop_orientation),
        ]

    def _check_pose_ik(self, pose, label=''):
        """Return True if `pose` has a valid IK solution; log a clear error if not."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = cfg.PLANNING_GROUP
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.is_diff = True
        req.ik_request.timeout.sec = 1
        ps = PoseStamped()
        ps.header.frame_id = cfg.BASE_FRAME
        ps.pose = pose
        req.ik_request.pose_stamped = ps

        self.ik_client.wait_for_service()
        future = self.ik_client.call_async(req)
        while not future.done():
            time.sleep(0.01)

        ok = future.result().error_code.val == MoveItErrorCodes.SUCCESS
        p = pose.position
        q = pose.orientation
        status = 'OK' if ok else 'UNREACHABLE'
        self.logger.info(
            f'[IK {status}] {label}  '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  '
            f'quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
        )
        return ok

    def move_upright_transport(self, waypoints, min_fraction=0.95):
        """Execute the full Cartesian transport path through all waypoints.

        Each waypoint carries its own orientation (grasp RPY → drop RPY),
        so compute_cartesian_path SLERPs orientation smoothly — no sudden
        EEF rotation during transport.

        Pre-checks IK for every waypoint endpoint so failures point directly
        to the arm_config.py value that needs adjustment.

        Returns True on success, False on failure.
        """
        names = ['raise to TRANSPORT_Z', 'mid-swing', 'above DROP_APPROACH', 'lower to DROP_APPROACH']

        # ── pre-flight IK check ────────────────────────────────────────
        all_ok = True
        for i, (wp, name) in enumerate(zip(waypoints, names)):
            if not self._check_pose_ik(wp, f'wp{i+1} {name}'):
                all_ok = False
        if not all_ok:
            self.logger.error(
                'One or more transport waypoints are IK-unreachable — '
                'calibrate TRANSPORT_Z / DROP_APPROACH_POSE in arm_config.py. '
                'See [IK UNREACHABLE] lines above for the exact pose.'
            )
            return False

        # ── plan Cartesian path ────────────────────────────────────────
        self.logger.info('Planning Cartesian transport path...')
        self.cartesian_path_client.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id = cfg.BASE_FRAME
        req.group_name = cfg.PLANNING_GROUP
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = cfg.CARTESIAN_MAX_STEP
        req.jump_threshold = cfg.CARTESIAN_JUMP_THRESHOLD
        req.avoid_collisions = True
        req.start_state.is_diff = True

        future = self.cartesian_path_client.call_async(req)
        while not future.done():
            time.sleep(0.01)

        result = future.result()
        frac = result.fraction
        self.logger.info(f'Cartesian transport path: {frac:.1%} planned')

        if frac < min_fraction:
            self.logger.error(
                f'Cartesian transport only {frac:.1%} complete (need {min_fraction:.0%}). '
                f'All waypoints passed IK but straight-line segments between them '
                f'hit unreachable configurations. '
                f'Try adjusting TRANSPORT_Z or adding an intermediate via-pose.'
            )
            return False

        if cfg.VIZ_BEFORE_EXEC:
            self._visualize_trajectory(result.solution)

        return self._execute_trajectory(result.solution)

    # ── deterministic planner function ────────────────────────────────────────────────────

    def _call_plan_joint(self, joint_angles, retries=3):
        """Sends a list of 7 joint angles to the MoveIt Joint Planner."""
        self.logger.info('Waiting for /xarm_joint_plan service...')
        self.plan_joint.wait_for_service()

        for attempt in range(1, retries + 1):
            req = PlanJoint.Request()
            req.target = joint_angles
            # req.velocity = 0.15 # Slower to speed. from 0.3 to 0.15
            future = self.plan_joint.call_async(req)
            while not future.done():
                time.sleep(0.05)

            if future.result() is not None and future.result().success:
                self.logger.info('Joint plan successful!')
                return True
            else:
                self.logger.warning(f'Joint plan attempt {attempt}/{retries} failed.')

        self.logger.error('Failed to generate joint plan after all retries.')
        return False

    def _call_plan_exec(self):
        req = PlanExec.Request()
        req.wait = True
        future = self.plan_exec.call_async(req)
        while not future.done():
            time.sleep(0.05)
        result = future.result()
        if not result.success:
            self.logger.error('PlanExec failed')
            return False
        return True