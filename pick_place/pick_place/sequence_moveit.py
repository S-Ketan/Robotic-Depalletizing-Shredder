"""Pick-and-place for the PiPER using MoveIt 2.

Big moves (home -> above object -> above drop spot) are planned by MoveIt.
The moves near the object (down, grasp, up) are straight lines of the
gripper (Cartesian / end-effector planning), so it doesn't swing sideways.
"""
import copy
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# ---- Change these values -------------------------------------------------
# Joint values in radians (joint1..joint6).
# Joint 6 = 0.63 on our arm: it has a hard stop at ~0.52 and can't reach 0.
HOME       = [0.00, 0.00,  0.00, 0.00, 0.00, 0.00]
ABOVE_PICK = [0.00, 1.60, -1.20, 0.00, 1.20, 0.00]   # x≈30, z≈25
ABOVE_DROP = [1.57, 1.60, -1.20, 0.00, 1.20, 0.00]   # turned 90° left

DOWN = 0.075     # how far the gripper goes straight down (meters)
OPEN = 0.07      # gripper fully open (meters)
CLOSED = 0.025   # a few mm LESS than the object's width - never 0 with an object!
SPEED = 0.1      # 10 % of max speed; keep it low on the real arm
# ---------------------------------------------------------------------------


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_moveit')
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # -- helpers ------------------------------------------------------------
    @staticmethod
    def wait_for(future):
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        return future.result()

    def run_action(self, client, goal):
        client.wait_for_server()
        handle = self.wait_for(client.send_goal_async(goal))
        if not handle.accepted:
            self.get_logger().error('MoveIt rejected the goal')
            return False
        result = self.wait_for(handle.get_result_async()).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'MoveIt error code {result.error_code.val}')
            return False
        return True

    def gripper_pose(self):
        """Where the gripper (tcp_link) is right now, relative to base_link."""
        t = self.tf_buffer.lookup_transform(
            'base_link', 'tcp_link', Time(), timeout=Duration(seconds=2.0))
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        return pose

    # -- obstacles ----------------------------------------------------------
    def add_box(self, name, size, position):
        """Add a box obstacle that MoveIt must not hit (sizes/positions in meters)."""
        box = CollisionObject()
        box.header.frame_id = 'base_link'
        box.id = name
        box.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        box.primitive_poses.append(pose)
        box.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(box)
        self.scene_client.wait_for_service()
        self.wait_for(self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)))

    # -- motions ------------------------------------------------------------
    def plan_to_joints(self, group, names, values, protect_j6=False):
        """Let MoveIt plan a smooth, collision-free move to joint values."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = SPEED
        req.max_acceleration_scaling_factor = SPEED
        req.start_state.is_diff = True
        target = Constraints()
        for name, value in zip(names, values):
            target.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        req.goal_constraints.append(target)
        if group == 'arm' and protect_j6:
            # Keep joint 6 between 0.6 and 2.97 during the WHOLE path
            # (our arm has a hard stop at ~0.52).
            req.path_constraints.joint_constraints.append(JointConstraint(
                joint_name='joint6', position=1.785,
                tolerance_above=1.18, tolerance_below=1.18, weight=1.0))
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return self.run_action(self.move_client, goal)

    def move_straight(self, dx=0.0, dy=0.0, dz=0.0):
        """Move the gripper in a straight line by dx/dy/dz meters."""
        target = copy.deepcopy(self.gripper_pose())
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'arm'
        req.link_name = 'tcp_link'
        req.start_state.is_diff = True
        req.waypoints = [target]
        req.max_step = 0.005          # compute a pose every 5 mm
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = SPEED
        req.max_acceleration_scaling_factor = SPEED

        self.cartesian_client.wait_for_service()
        res = self.wait_for(self.cartesian_client.call_async(req))
        if res.fraction < 0.99:
            self.get_logger().error(
                f'Straight line only possible for {res.fraction * 100:.0f} % '
                'of the way (out of reach or a joint limit). Try another pose.')
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        return self.run_action(self.exec_client, goal)

    def arm_to(self, joints, protect_j6=False):
        return self.plan_to_joints('arm', ARM_JOINTS, joints, protect_j6)

    def gripper_to(self, width):
        if not self.plan_to_joints('gripper', ['gripper'], [width]):
            # When an object is between the fingers they can't reach the exact
            # width - that's expected, so we only warn and continue.
            self.get_logger().warn('Gripper did not reach the exact width '
                                   '(normal when holding an object) - continuing')
        return True

    # -- the task -----------------------------------------------------------
    def run(self):
        self.get_logger().info('Waiting for MoveIt ...')
        self.move_client.wait_for_server()
        self.gripper_pose()  # wait until the robot's position is known
        # Table: 1.2 m x 1.2 m, 4 cm thick, top surface 5 mm below the arm's base
        self.add_box('table', size=(1.2, 1.2, 0.04), position=(0.2, 0.0, -0.025))

        steps = [
            ('Going home',           lambda: self.arm_to(HOME, protect_j6=False)),
            ('Opening gripper',      lambda: self.gripper_to(OPEN)),
            ('Above object',         lambda: self.arm_to(ABOVE_PICK)),
            ('Straight down',        lambda: self.move_straight(dz=-DOWN)),
            ('Grasping',             lambda: self.gripper_to(CLOSED)),
            ('Straight up',          lambda: self.move_straight(dz=+DOWN)),
            ('Moving to drop spot',  lambda: self.arm_to(ABOVE_DROP)),
            ('Releasing',            lambda: self.gripper_to(OPEN)),
            ('Back home',            lambda: self.arm_to(HOME)),
        ]
        for name, action in steps:
            self.get_logger().info(name)
            if not action():
                self.get_logger().error(f'Step "{name}" failed - stopping.')
                return
        self.get_logger().info('Done!')


def main():
    rclpy.init()
    node = PickPlace()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
