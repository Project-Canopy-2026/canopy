#planner_node.py
import math
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


class ManipulationPlanner(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        # grasp orientation (point A)
        gr, gp, gy = 90.0, -90.0, 0.0
        # drop orientation (point B)
        dr, dp, dy = 90.0, -90.0, 90.0

        self.wpA_joints = self._deg_to_rad(-90.0, 0.00, 0.0, 10.0, -180.0,  80.0, 180.0)
        self.wpB_joints = self._deg_to_rad( 0.00,-60.0, 0.0, 60.0, -180.0, -30.0, 180.0)
        self.grasp_pose = self._make_pose(0.0, -0.450, 0.160,  gr, gp, gy)
        # self.pre_grasp_pose = self._make_pose(0.3, 0.0, 0.4, 0.0, 0.0, 0.0)
        # self.grasp_pose     = self._make_pose(0.0, -0.500, 0.200,  gr, gp, gy)
        self.lift_pose      = self._make_pose(0.0, -0.400, 0.550,  gr, gp, gy)
        self.drop_pose      = self._make_pose( 0.500, -0.2000, 0.550, dr, dp, dy)

        self.plan_pose     = self.create_client(PlanPose, '/xarm_pose_plan')
        self.plan_joint    = self.create_client(PlanJoint, '/xarm_joint_plan')
        self.plan_exec     = self.create_client(PlanExec, '/xarm_exec_plan')
        self.gripper_open  = self.create_client(Trigger,  '/gripper/open')
        self.gripper_close = self.create_client(Trigger,  '/gripper/close')

        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)

        self.get_logger().info('Planner node started. Waiting for services...')
        self._wait_for_services()
        self.get_logger().info('All services ready.')

    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        qx = math.sin(r/2)*math.cos(p/2)*math.cos(y/2) - math.cos(r/2)*math.sin(p/2)*math.sin(y/2)
        qy = math.cos(r/2)*math.sin(p/2)*math.cos(y/2) + math.sin(r/2)*math.cos(p/2)*math.sin(y/2)
        qz = math.cos(r/2)*math.cos(p/2)*math.sin(y/2) - math.sin(r/2)*math.sin(p/2)*math.cos(y/2)
        qw = math.cos(r/2)*math.cos(p/2)*math.cos(y/2) + math.sin(r/2)*math.sin(p/2)*math.sin(y/2)
        return qx, qy, qz, qw
    
    def _deg_to_rad(self, j1, j2, j3, j4, j5, j6, j7):
        return [
            math.radians(float(j1)), 
            math.radians(float(j2)), 
            math.radians(float(j3)), 
            math.radians(float(j4)), 
            math.radians(float(j5)), 
            math.radians(float(j6)), 
            math.radians(float(j7))
        ]


    def _make_pose(self, x, y, z, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = self.euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _make_box_object(self, name, x, y, z, lx, ly, lz):
        obj = CollisionObject()
        obj.header.frame_id = 'link_base'
        obj.id = name
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [lx, ly, lz]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def setup_collision_scene(self):
        self.get_logger().info('Setting up collision scene...')
        scene = PlanningScene()
        scene.is_diff = True
        # floor plane — prevents arm going below base level
        scene.world.collision_objects.append(
            self._make_box_object('floor', x=0.0, y=0.0, z=-0.05, lx=3.0, ly=3.0, lz=0.05))
        scene.world.collision_objects.append(
            self._make_box_object('rail_left',  x=-0.30, y=0.35,  z=0.085, lx=1.40, ly=0.29, lz=0.17))
        scene.world.collision_objects.append(
            self._make_box_object('rail_right', x=-0.30, y=-0.7, z=0.085, lx=1.40, ly=0.29, lz=0.17))
        scene.world.collision_objects.append(
            self._make_box_object('planting_assembly', x=0.450, y= 0.0, z=0.2, lx=0.2, ly=1.0, lz=0.5))        
        # scene.world.collision_objects.append(
            # self._make_box_object('pole',       x=0.24,  y=-0.24, z=0.60,  lx=0.06, ly=0.06, lz=1.20))
        for _ in range(5):
            self.scene_pub.publish(scene)
            time.sleep(0.5)
        self.get_logger().info('Collision scene ready.')

    def _wait_for_services(self):
        for client in [
            self.plan_pose,
            self.plan_joint,
            self.plan_exec,
            self.gripper_open,
            self.gripper_close,
        ]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Waiting for {client.srv_name}...')

    def _call_plan_pose(self, pose):
        req = PlanPose.Request()
        req.target = pose
        future = self.plan_pose.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.success:
            self.get_logger().error('PlanPose failed')
            return False
        return True

    def _call_plan_joint(self, joint_angles):
        """Sends a list of 7 joint angles to the MoveIt Joint Planner."""
        self.get_logger().info('Waiting for /xarm_joint_plan service...')
        self.plan_joint.wait_for_service()
        
        req = PlanJoint.Request()
        req.target = joint_angles
        
        # Send the request
        future = self.plan_joint.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info('Joint plan successful!')
            return True
        else:
            self.get_logger().error('Failed to generate joint plan.')
            return False

    def _call_plan_exec(self):
        req = PlanExec.Request()
        req.wait = True
        future = self.plan_exec.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.success:
            self.get_logger().error('PlanExec failed')
            return False
        return True

    def _call_gripper(self, client, action):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.success:
            self.get_logger().error(f'Gripper {action} failed: {result.message}')
            return False
        self.get_logger().info(f'Gripper {action} success')
        return True

    def run(self):
        self.get_logger().info('Starting pick and place sequence...')

        self.setup_collision_scene()

        self.get_logger().info('Step 0: Moving to Waypoint A')
        if not self._call_plan_joint(self.wpA_joints):
            return
        if not self._call_plan_exec():
            return
        time.sleep(1.0)

        # self.get_logger().info('Step 1: Moving to pre-grasp...')
        # if not self._call_plan_pose(self.pre_grasp_pose):
        #     return
        # if not self._call_plan_exec():
        #     return
        
        # time.sleep(1.0)

        self.get_logger().info('Step 2: Opening gripper...')
        if not self._call_gripper(self.gripper_open, 'open'):
            return
        
        time.sleep(1.0)

        self.get_logger().info('Step 3: Moving towards to grasp...')
        if not self._call_plan_pose(self.grasp_pose):
            return
        if not self._call_plan_exec():
            return
        
        time.sleep(1.0)

        self.get_logger().info('Step 4: Closing gripper...')
        if not self._call_gripper(self.gripper_close, 'close'):
            return
        
        time.sleep(1.0)

        self.get_logger().info('Step 5: Lifting...')
        if not self._call_plan_pose(self.lift_pose):
            return
        if not self._call_plan_exec():
            return
        
        time.sleep(1.0)

        self.get_logger().info('Step 5.5: Moving to Waypoint B')
        if not self._call_plan_joint(self.wpB_joints):
            return
        if not self._call_plan_exec():
            return
        
        time.sleep(1.0)

        self.get_logger().info('Step 6: Moving to drop waypoint...')
        if not self._call_plan_pose(self.drop_pose):
            return
        if not self._call_plan_exec():
            return
        
        time.sleep(1.0)

        self.get_logger().info('Step 7: Releasing object...')
        if not self._call_gripper(self.gripper_open, 'open'):
            return
        
        time.sleep(1.0)

        self.get_logger().info('Pick and place complete.')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationPlanner()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()