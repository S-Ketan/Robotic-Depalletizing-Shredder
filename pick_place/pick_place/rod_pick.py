"""Phase 3: pick up an elongated object lying on the table, carry it and put it down.

The object lies sideways (along y) in front of the arm. Joint 6 turns the gripper
90° so the fingers close ACROSS it. The object is added to MoveIt and attached
to the gripper while it is carried, so MoveIt also checks it for collisions.

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

# ---- Object (change these) ------------------------------------------------
SHAPE = 'box'              # 'box' = flat cuboid (e.g. spirit level), 'rod' = round rod

ROD_X, ROD_Y = 0.305, 0.0  # centre of the object on the table (m from the arm base)

if SHAPE == 'rod':
    ## Round rod: measure and enter the values here (m)
    ROD_LENGTH = 0.25
    ROD_DIAMETER = 0.02
    ROD_HEIGHT = ROD_DIAMETER           # lying on the table
    ROD_THICK = ROD_DIAMETER            # the side the fingers squeeze
else:
    ## Flat cuboid, standing on its narrow edge: measure and enter the values here (m)
    ROD_LENGTH = 0.41      # long side
    ROD_HEIGHT = 0.05      # how tall it stands
    ROD_THICK  = 0.02     # thickness, the side the fingers squeeze

CLOSED = ROD_THICK - 0.004            # a few mm less than the object - never 0!
DOWN = 0.115 - ROD_HEIGHT / 2         # fingertips from ~11.5 cm down to the object's mid-height

# ---- Poses ----------------------------------------------------------------
J6 = 1.57                  # gripper turned 90° -> fingers close across the object
HOME       = [0.00, 0.00,  0.00, 0.00, 0.00, 0.00]
ABOVE_PICK = [0.00, 1.60, -1.20, 0.00, 1.20, J6]   # fingertips ~11.5 cm above table
ABOVE_DROP = [1.57, 1.60, -1.20, 0.00, 1.20, J6]   # turned 90° left

OPEN = 0.07

# Gripper parts that may touch the object while it is held
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
        """Put the object into MoveIt's world, lying on the table along y."""
        rod = CollisionObject()
        rod.header.frame_id = 'base_link'
        rod.id = 'rod'
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = ROD_X, ROD_Y, ROD_HEIGHT / 2
        if SHAPE == 'rod':
            rod.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[ROD_LENGTH, ROD_DIAMETER / 2]))
            # A cylinder stands upright by default: rotate it 90° about x so it lies along y
            pose.orientation.x = 0.7071068
            pose.orientation.w = 0.7071068
        else:
            # Box: x = thickness (front-back), y = length (left-right), z = height
            rod.primitives.append(SolidPrimitive(
                type=SolidPrimitive.BOX, dimensions=[ROD_THICK, ROD_LENGTH, ROD_HEIGHT]))
            pose.orientation.w = 1.0
        rod.primitive_poses.append(pose)
        rod.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(rod)
        return self.apply_scene(scene)

    def attach_rod(self):
        """Tell MoveIt the object now moves with the gripper."""
        held = AttachedCollisionObject()
        held.link_name = 'gripper_base'
        held.object.id = 'rod'            # takes the object from the world
        held.object.operation = CollisionObject.ADD
        held.touch_links = TOUCH_LINKS
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects.append(held)
        return self.apply_scene(scene)

    def detach_rod(self):
        """Tell MoveIt the object is released (it stays in the world where it is)."""
        held = AttachedCollisionObject()
        held.link_name = 'gripper_base'
        held.object.id = 'rod'
        held.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects.append(held)
        return self.apply_scene(scene)

    def run(self):
        self.get_logger().info(f'Waiting for MoveIt ... (object: {SHAPE})')
        self.move_client.wait_for_server()
        self.gripper_pose()  # wait until the robot's position is known
        self.add_box('table', size=(1.2, 1.2, 0.04), position=(0.2, 0.0, -0.025))
        self.detach_rod()    # clean up if an earlier run stopped while holding the object
        self.add_rod()

        steps = [
            ('Going home',                        lambda: self.arm_to(HOME)),
            ('Opening gripper',                   lambda: self.gripper_to(OPEN)),
            ('Above object, gripper turned 90°',  lambda: self.arm_to(ABOVE_PICK)),
            ('Straight down to object',           lambda: self.move_straight(dz=-DOWN)),
            ('Attach object in MoveIt',           self.attach_rod),
            ('Grasping',                          lambda: self.gripper_to(CLOSED)),
            ('Straight up',                       lambda: self.move_straight(dz=+DOWN)),
            ('Carrying to drop spot',             lambda: self.arm_to(ABOVE_DROP)),
            ('Lowering',                          lambda: self.move_straight(dz=-(DOWN - 0.01))),
            ('Releasing',                         lambda: self.gripper_to(OPEN)),
            ('Detach object in MoveIt',           self.detach_rod),
            ('Straight up',                       lambda: self.move_straight(dz=+(DOWN - 0.01))),
            ('Back home',                         lambda: self.arm_to(HOME)),
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

