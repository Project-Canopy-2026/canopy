import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, Constraints,
    OrientationConstraint, PositionConstraint, BoundingVolume,
    RobotState, JointConstraint
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
import threading


class ManipulationPlanner(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        # ── orientations ───────────────────────────────────────────────
        gr, gp, gy = 89.7, -90.0, 0.0       # grasp orientation
        dr, dp, dy = 96.7, -87.2, 85.1       # drop orientation

        # ── joint goals (deterministic) ────────────────────────────────
        self.pre_grasp_joints = self._deg_to_rad(-61.1, -3.7, -30.5, 2.0, -181.5, 84.7, -91.7)
        self.drop_joints      = self._deg_to_rad(-35.0, -25.0, 0.0, 75.0, -160.0, -10.0, -110.0)

        # ── cartesian goals (perception-driven / Pilz) ─────────────────
        self.grasp_pose = self._make_pose(0.0, -0.450, 0.20, gr, gp, gy)
        # lift is same XY as grasp, just Z+150mm straight up
        self.lift_pose  = self._make_pose(0.0, -0.400, 0.550, gr, gp, gy)

        # ── joint names ────────────────────────────────────────────────
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint7'
        ]

        # ── action / service clients ───────────────────────────────────
        self.move_client   = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_open  = self.create_client(Trigger, '/gripper/open')
        self.gripper_close = self.create_client(Trigger, '/gripper/close')

        # ── planning scene publisher ───────────────────────────────────
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # ── trigger service ────────────────────────────────────────────
        self.should_run = False
        self.trigger_srv = self.create_service(Trigger, '/planner/trigger', self.trigger_cb)

        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_client.wait_for_server()
        self.get_logger().info('Waiting for gripper services...')
        self._wait_for_services()
        self.get_logger().info('All ready. Call /planner/trigger to start.')

    # ── helpers ────────────────────────────────────────────────────────
    def trigger_cb(self, request, response):
        self.should_run = True
        response.success = True
        response.message = 'Sequence triggered'
        return response

    def _deg_to_rad(self, j1, j2, j3, j4, j5, j6, j7):
        return [math.radians(float(v)) for v in [j1, j2, j3, j4, j5, j6, j7]]

    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        qx = math.sin(r/2)*math.cos(p/2)*math.cos(y/2) - math.cos(r/2)*math.sin(p/2)*math.sin(y/2)
        qy = math.cos(r/2)*math.sin(p/2)*math.cos(y/2) + math.sin(r/2)*math.cos(p/2)*math.sin(y/2)
        qz = math.cos(r/2)*math.cos(p/2)*math.sin(y/2) - math.sin(r/2)*math.sin(p/2)*math.cos(y/2)
        qw = math.cos(r/2)*math.cos(p/2)*math.cos(y/2) + math.sin(r/2)*math.sin(p/2)*math.sin(y/2)
        return qx, qy, qz, qw

    def _make_pose(self, x, y, z, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
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

    # ── collision scene ────────────────────────────────────────────────
    def _make_box(self, name, x, y, z, lx, ly, lz):
        obj = CollisionObject()
        obj.header.frame_id = 'link_base'
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
        return obj

    def setup_collision_scene(self):
        self.get_logger().info('Setting up collision scene...')
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            self._make_box('floor',      0.0,   0.0,  -0.05, 3.0,  3.0,  0.05))
        scene.world.collision_objects.append(
            self._make_box('rail_left',  -0.30,  0.56,  0.085, 1.40, 0.29, 0.17))
        scene.world.collision_objects.append(
            self._make_box('rail_right', -0.30, -0.56,  0.085, 1.40, 0.29, 0.17))
        scene.world.collision_objects.append(
            self._make_box('pole',        0.24, -0.24,  0.60,  0.06, 0.06, 1.20))
        for _ in range(5):
            self.scene_pub.publish(scene)
            time.sleep(0.2)
        self.get_logger().info('Collision scene ready.')

    # ── upright constraint ─────────────────────────────────────────────
    def _upright_constraint(self):
        oc = OrientationConstraint()
        oc.header.frame_id = 'link_base'
        oc.link_name = 'link_eef'
        oc.orientation = self.grasp_pose.orientation
        oc.absolute_x_axis_tolerance = 0.26
        oc.absolute_y_axis_tolerance = 0.26
        oc.absolute_z_axis_tolerance = 6.28
        oc.weight = 1.0
        c = Constraints()
        c.name = 'upright'
        c.orientation_constraints.append(oc)
        return c

    # ── send goal helper ───────────────────────────────────────────────
    def _send_goal(self, goal, retries=3):
        for attempt in range(retries):
            future = self.move_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal rejected, attempt {attempt+1}/{retries}')
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return True

            self.get_logger().warn(
                f'Move failed (code {result.error_code.val}), '
                f'attempt {attempt+1}/{retries}')

        self.get_logger().error('Move failed after all retries')
        return False

    # ── move to joint angles (deterministic) ───────────────────────────
    def _move_joints(self, joint_angles, retries=3):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'xarm7'
        req.num_planning_attempts = 3
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
        req.start_state.is_diff = True

        joint_constraints = Constraints()
        for name, angle in zip(self.joint_names, joint_angles):
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
    def _move_to(self, pose, pipeline='ompl', planner='RRTConnect',
                 constrained=False, retries=3):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'xarm7'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.start_state.is_diff = True

        # position constraint
        pc = PositionConstraint()
        pc.header.frame_id = 'link_base'
        pc.link_name = 'link_eef'
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

        # orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = 'link_base'
        oc.link_name = 'link_eef'
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)
        goal_constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(goal_constraints)

        if constrained:
            req.path_constraints = self._upright_constraint()

        return self._send_goal(goal, retries)

    def _wait_for_services(self):
        for client in [self.gripper_open, self.gripper_close]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Waiting for {client.srv_name}...')

    def _call_gripper(self, client, action):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.success:
            self.get_logger().error(f'Gripper {action} failed')
            return False
        self.get_logger().info(f'Gripper {action} success')
        return True

    # ── main sequence ──────────────────────────────────────────────────
    def run(self):
        while rclpy.ok():
            if self.should_run:
                self.should_run = False
                self._execute_sequence()
            else:
                time.sleep(0.1)

    def _execute_sequence(self):
        self.get_logger().info('=== Starting pick and place ===')

        self.setup_collision_scene()

        # Step 1: joint-space move to pre-grasp (deterministic)
        self.get_logger().info('Step 1: Pre-grasp (joint)...')
        if not self._move_joints(self.pre_grasp_joints):
            return

        # Step 2: open gripper
        self.get_logger().info('Step 2: Open gripper...')
        if not self._call_gripper(self.gripper_open, 'open'):
            return

        # Step 3: Pilz LIN straight-line to grasp
        self.get_logger().info('Step 3: Grasp descent (Pilz LIN)...')
        if not self._move_to(self.grasp_pose,
                             pipeline='pilz_industrial_motion_planner',
                             planner='LIN'):
            self.get_logger().warn('Pilz LIN failed, falling back to OMPL...')
            if not self._move_to(self.grasp_pose):
                return

        # Step 4: close gripper
        self.get_logger().info('Step 4: Close gripper...')
        if not self._call_gripper(self.gripper_close, 'close'):
            return

        # Step 5: Pilz LIN straight up (lift)
        self.get_logger().info('Step 5: Lift (Pilz LIN)...')
        if not self._move_to(self.lift_pose,
                             pipeline='pilz_industrial_motion_planner',
                             planner='LIN',
                             constrained=True):
            self.get_logger().warn('Pilz LIN lift failed, falling back to OMPL...')
            if not self._move_to(self.lift_pose, constrained=True):
                return

        # Step 6: joint-space move to drop (deterministic, constrained)
        self.get_logger().info('Step 6: Drop (joint)...')
        if not self._move_joints(self.drop_joints):
            return

        # Step 7: release
        self.get_logger().info('Step 7: Release...')
        if not self._call_gripper(self.gripper_open, 'open'):
            return

        self.get_logger().info('=== Pick and place complete ===')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationPlanner()
    run_thread = threading.Thread(target=node.run, daemon=True)
    run_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()