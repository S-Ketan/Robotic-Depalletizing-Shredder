"""Phase 3: pick up a rod lying on the table, carry it and put it down.

The rod lies sideways (along y) in front of the arm. Joint 6 turns the gripper
90° so the fingers close ACROSS the rod. The rod is added to MoveIt as an
object and attached to the gripper while it is carried, so MoveIt also checks
the rod for collisions.

Simulation:  piper_moveit        +  ros2 run pick_place rod_pick
Real arm:    piper_real_moveit   +  ros2 run pick_place rod_pick
"""
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from pick_place.sequence_moveit import PickPlace, SETTLE

# ---- Rod and poses (change these) -----------------------------------------
## for a rod : measure and enter the values here
ROD_LENGTH = 0.25          # m
ROD_DIAMETER = 0.02        # m
ROD_X, ROD_Y = 0.305, 0.0  # centre of the rod on the table (m from the arm base)

## For a flat cuboid: measure and enter the dimensions here

# ROD_LENGTH = 0.60      # long side (m)
# ROD_HEIGHT = 0.05      # how tall it stands (m)
# ROD_THICK  = 0.025     # thickness, the side the fingers squeeze (m)
# CLOSED = ROD_THICK - 0.004
# DOWN = 0.115 - ROD_HEIGHT / 2   # fingertips go down to the middle of its height

J6 = 1.57                  # gripper turned 90° -> fingers close across the rod
HOME       = [0.00, 0.00,  0.00, 0.00, 0.00, 0.00]
ABOVE_PICK = [0.00, 1.60, -1.20, 0.00, 1.20, J6]   # fingertips ~11.5 cm above table
ABOVE_DROP = [1.57, 1.60, -1.20, 0.00, 1.20, J6]   # turned 90° left

DOWN = 0.105               # fingertips from ~11.5 cm down to ~1 cm (rod centre)
OPEN = 0.07
CLOSED = ROD_DIAMETER - 0.004   # a few mm less than the rod - never 0!

# Gripper parts that may touch the rod while it is held
TOUCH_LINKS = ['link6', 'flange_link', 'gripper_base', 'gripper_link1', 'gripper_link2']
# ---------------------------------------------------------------------------


class RodPick(PickPlace):
    def apply_scene(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        self.scene_client.wait_for_service()
        self.wait_for(self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)))
        return True

    def add_rod(self):
        """Put the rod into MoveIt's world, lying on the table along y."""
        rod = CollisionObject()
        rod.header.frame_id = 'base_link'
        rod.id = 'rod'
        rod.primitives.append(SolidPrimitive(
            type=SolidPrimitive.CYLINDER, dimensions=[ROD_LENGTH, ROD_DIAMETER / 2]))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = ROD_X, ROD_Y, ROD_DIAMETER / 2
        # A cylinder stands upright by default: rotate it 90° about x so it lies along y
        pose.orientation.x = 0.7071068
        pose.orientation.w = 0.7071068
        rod.primitive_poses.append(pose)
        rod.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(rod)
        return self.apply_scene(scene)

    def attach_rod(self):
        """Tell MoveIt the rod now moves with the gripper."""
        held = AttachedCollisionObject()
        held.link_name = 'gripper_base'
        held.object.id = 'rod'            # takes the rod from the world
        held.object.operation = CollisionObject.ADD
        held.touch_links = TOUCH_LINKS
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects.append(held)
        return self.apply_scene(scene)

    def detach_rod(self):
        """Tell MoveIt the rod is released (it stays in the world where it is)."""
        held = AttachedCollisionObject()
        held.link_name = 'gripper_base'
        held.object.id = 'rod'
        held.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects.append(held)
        return self.apply_scene(scene)

    def run(self):
        self.get_logger().info('Waiting for MoveIt ...')
        self.move_client.wait_for_server()
        self.gripper_pose()  # wait until the robot's position is known
        self.add_box('table', size=(1.2, 1.2, 0.04), position=(0.2, 0.0, -0.025))
        self.detach_rod()    # clean up if an earlier run stopped while holding the rod
        self.add_rod()

        steps = [
            ('Going home',                     lambda: self.arm_to(HOME)),
            ('Opening gripper',                lambda: self.gripper_to(OPEN)),
            ('Above rod, gripper turned 90°',  lambda: self.arm_to(ABOVE_PICK)),
            ('Straight down to rod',           lambda: self.move_straight(dz=-DOWN)),
            ('Attach rod in MoveIt',           self.attach_rod),
            ('Grasping',                       lambda: self.gripper_to(CLOSED)),
            ('Straight up',                    lambda: self.move_straight(dz=+DOWN)),
            ('Carrying to drop spot',          lambda: self.arm_to(ABOVE_DROP)),
            ('Lowering',                       lambda: self.move_straight(dz=-(DOWN - 0.01))),
            ('Releasing',                      lambda: self.gripper_to(OPEN)),
            ('Detach rod in MoveIt',           self.detach_rod),
            ('Straight up',                    lambda: self.move_straight(dz=+(DOWN - 0.01))),
            ('Back home',                      lambda: self.arm_to(HOME)),
        ]
        for name, action in steps:
            self.get_logger().info(name)
            if not action():
                self.get_logger().error(f'Step "{name}" failed - stopping.')
                return
            time.sleep(SETTLE)
        self.get_logger().info('Done!')


def main():
    rclpy.init()
    node = RodPick()
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
