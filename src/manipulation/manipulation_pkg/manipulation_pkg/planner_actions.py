import math
import time
import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, Constraints,
    OrientationConstraint, PositionConstraint,
    BoundingVolume, JointConstraint
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive

from manipulation_pkg import robot_config as cfg


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

        self.logger.info('Waiting for MoveGroup action server...')
        self.move_client.wait_for_server()
        self._wait_for_services()
        self.logger.info('Planner ready.')

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

    # ── send goal ──────────────────────────────────────────────────────
    def _send_goal(self, goal, retries=None):
        retries = retries or cfg.PLAN_RETRIES
        for attempt in range(retries):
            future = self.move_client.send_goal_async(goal)
            while not future.done():
                time.sleep(0.01)
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.logger.warn(f'Goal rejected, attempt {attempt+1}/{retries}')
                continue

            result_future = goal_handle.get_result_async()
            while not result_future.done():
                time.sleep(0.01)
            result = result_future.result().result

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return True

            self.logger.warn(
                f'Move failed (code {result.error_code.val}), attempt {attempt+1}/{retries}'
                f' | pipeline={goal.request.pipeline_id} planner={goal.request.planner_id}'
            )

        self.logger.error('Move failed after all retries')
        return False

    # ── move to joint angles ───────────────────────────────────────────
    def move_joints(self, joint_angles_deg, retries=None):
        joint_angles_rad = self.deg_to_rad(joint_angles_deg)
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = cfg.PLANNING_GROUP
        req.num_planning_attempts = cfg.NUM_PLANNING_ATTEMPTS
        req.allowed_planning_time = cfg.PLANNING_TIME
        req.max_velocity_scaling_factor = cfg.MAX_VELOCITY_SCALING
        req.max_acceleration_scaling_factor = cfg.MAX_ACCELERATION_SCALING
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
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

    # ── move to cartesian pose ─────────────────────────────────────────
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

        # orientation constraint (goal)
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

    def _call_gripper(self, client, action):
        req = Trigger.Request()
        future = client.call_async(req)
        while not future.done():
            time.sleep(0.01)
        result = future.result()
        if not result.success:
            self.logger.error(f'Gripper {action} failed')
            return False
        self.logger.info(f'Gripper {action} success')
        return True