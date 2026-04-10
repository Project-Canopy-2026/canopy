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
    BoundingVolume, JointConstraint, RobotState
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
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

        self.logger.info('Waiting for MoveGroup action server...')
        self.move_client.wait_for_server()
        self._wait_for_services()
        self.logger.info('Planner ready.')

        self.ee_link = "tool_tcp"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')
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

        return list(result.solution.joint_state.position)


    # ── deterministic planner function ────────────────────────────────────────────────────

    def _call_plan_joint(self, joint_angles):
        """Sends a list of 7 joint angles to the MoveIt Joint Planner."""
        self.logger.info('Waiting for /xarm_joint_plan service...')
        self.plan_joint.wait_for_service()
        
        req = PlanJoint.Request()
        req.target = joint_angles
        
        future = self.plan_joint.call_async(req)
        while not future.done():
            time.sleep(0.05)

        if future.result() is not None and future.result().success:
            self.logger.info('Joint plan successful!')
            return True
        else:
            self.logger.error('Failed to generate joint plan.')
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