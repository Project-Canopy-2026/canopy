import time
from manipulation_pkg import robot_config as cfg
from geometry_msgs.msg import Pose


class PickAndPlaceExecutor:
    def __init__(self, planner):
        self.planner = planner
        self.logger = planner.logger

        # build poses from config
        self.grasp_pose = planner.make_pose_from_dict(cfg.GRASP_POSE)
        # self.lift_pose  = planner.make_pose_from_dict(cfg.LIFT_POSE)

        # lift: same X,Y,orientation as grasp — only Z changes
        self.lift_pose = planner.make_pose(
            cfg.GRASP_POSE['x'],
            cfg.GRASP_POSE['y'],
            cfg.LIFT_POSE['z']
        )
        self.lift_pose.orientation = self.grasp_pose.orientation
    
    def test_move_straight(self):
        self.logger.info('========== TESTING move_straight() ==========')

        # Move to a known safe joint pose first
        self.logger.info('Moving to PRE_GRASP_JOINTS_DEG first...')
        if not self.planner.move_joints(cfg.PRE_GRASP_JOINTS_DEG):
            self.logger.error('Failed to reach pre-grasp joint pose')
            return False

        # Get current end effector pose
        current_pose = self.planner.get_current_eef_pose()
        if current_pose is None:
            self.logger.error('Could not get current EEF pose')
            return False

        self.logger.info(
            f'Current pose: x={current_pose.position.x:.3f}, '
            f'y={current_pose.position.y:.3f}, '
            f'z={current_pose.position.z:.3f}'
        )

        # Create a tiny upward move: +1 cm in Z
        test_pose = Pose()
        test_pose.position.x = current_pose.position.x
        test_pose.position.y = current_pose.position.y
        test_pose.position.z = current_pose.position.z + 0.01
        test_pose.orientation = current_pose.orientation

        self.logger.info(
            f'Testing move_straight to: x={test_pose.position.x:.3f}, '
            f'y={test_pose.position.y:.3f}, '
            f'z={test_pose.position.z:.3f}'
        )

        success = self.planner.move_straight(test_pose, step=0.002)

        if success:
            self.logger.info('✅ move_straight test PASSED')
        else:
            self.logger.error('❌ move_straight test FAILED')

        return success

    def run(self):
        return self.test_move_straight()

    
    
    # def run(self):
    
    #     self.logger.info('=================== Starting pick and place ===================')

    #     # set up collision scene first
    #     self.planner.setup_collision_scene()

    #     # Step 1: joint-space to pre-grasp (deterministic)
    #     self.logger.info('Step 1: Pre-grasp (joint)...')
    #     if not self.planner.move_joints(cfg.PRE_GRASP_JOINTS_DEG):
    #         self.logger.error('Failed at step 1')
    #         return False
        
    #     # Step 1.5: get current EEF pose via FK
    #     current_pose = self.planner.get_current_eef_pose()
    #     if current_pose is None:
    #         self.logger.error('Failed to get current EEF pose')
    #         return False

    #     # Step 2: open gripper
    #     self.logger.info('Step 2: Open gripper...')
    #     if not self.planner.open_gripper():
    #         self.logger.error('Failed at step 2')
    #         return False
        

    #     # # Step 2.5: align orientation at current position with grasp orientation
    #     # self.logger.info('*************Step 2.5: Align orientation...**************')
    #     # align_pose = Pose()
    #     # align_pose.position.x = current_pose.position.x
    #     # align_pose.position.y = current_pose.position.y
    #     # align_pose.position.z = current_pose.position.z
    #     # align_pose.orientation = self.grasp_pose.orientation  # use grasp orientation
    #     # if not self.planner.move_cartesian(
    #     #     align_pose,
    #     #     pipeline='pilz_industrial_motion_planner',
    #     #     planner='PTP'
    #     # ):
    #     #     self.logger.error('Failed at orientation align')
    #     #     return False
    

    #     # Step 3: grasp approach
    #     self.logger.info('Step 3: Grasp Move (Pilz LIN)...')
    #     success = self.planner.move_cartesian(
    #         self.grasp_pose,
    #         pipeline='pilz_industrial_motion_planner',
    #         planner='LIN'
    #     )
    #     if not success:
    #         self.logger.warn('Pilz LIN failed, falling back to OMPL...')
    #         success = self.planner.move_cartesian(self.grasp_pose)
    #     if not success:
    #         self.logger.error('Failed at step 3')
    #         return False


    #     # # Step 3: straight-line to grasp
    #     # self.logger.info('Step 3: Grasp Move (straight line)...')
    #     # success = self.planner.move_cartesian(self.grasp_pose)
    #     # if not success:
    #     #     self.logger.warn('Straight line grasp, falling back to OMPL...')
    #     #     success = self.planner.move_cartesian(self.grasp_pose)
    #     # if not success:
    #     #     self.logger.error('Failed at step 3'); return False

    #     # Step 4: close gripper
    #     self.logger.info('Step 4: Close gripper...')
    #     if not self.planner.close_gripper():
    #         self.logger.error('Failed at step 4')
    #         return False

    #     #  # Step 5: Lift up
    #     # self.logger.info('Step 5: Lift (straight line)...')
    #     # success = self.planner.move_cartesian(self.lift_pose)
    #     # # if not success:
    #     # #     self.logger.warn('Straight lift failed, falling back to OMPL...')
    #     # #     success = self.planner.move_cartesian(self.lift_pose)
    #     # if not success:
    #     #     self.logger.error('Failed at step 5'); return False



    #     # Step 5: lift
    #     self.logger.info('Step 5: Lift (Pilz PTP)...')
    #     success = self.planner.move_cartesian(
    #         self.lift_pose,
    #         pipeline='pilz_industrial_motion_planner',
    #         planner='PTP'
    #     )
    #     if not success:
    #         self.logger.warn('Pilz PTP failed, falling back to OMPL...')
    #         success = self.planner.move_cartesian(self.lift_pose)
    #     if not success:
    #         self.logger.error('Failed at step 5')
    #         return False

    #     # Step 6: joint-space to drop (deterministic)
    #     self.logger.info('Step 6: Drop (joint)...')
    #     if not self.planner.move_joints(cfg.DROP_JOINTS_DEG):
    #         self.logger.error('Failed at step 6')
    #         return False

    #     # Step 7: release
    #     self.logger.info('Step 7: Release...')
    #     if not self.planner.open_gripper():
    #         self.logger.error('Failed at step 7')
    #         return False

        # self.logger.info('=== Pick and place complete ===')
        # return True