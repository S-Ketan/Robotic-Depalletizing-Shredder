"""Phase 4: pick up the elongated object, lift it and tilt it 45°.

The object lies left-right in front of the arm, exactly as in rod_pick.py, and is
picked from above. Then the arm turns the gripper to point FORWARD, which leaves
the object horizontal. Finally joint 6 rolls the gripper by 45°: because the
gripper now points straight forward, this tilts the object 45° around its own
middle (one side end down). Then everything is undone and the object is put
back where it was.

Why this way: with the gripper pointing down, joint 5 is already near its limit
(±70°), so the PiPER cannot tilt the object by 45° from that pose. Pointing
forward and rolling with joint 6 avoids that limit.

Reuses the object size and settings from rod_pick.py (change them there).

Simulation:  piper_moveit        +  ros2 run pick_place rod_tilt
Real arm:    piper_real_moveit   +  ros2 run pick_place rod_tilt
"""
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from pick_place.sequence_moveit import SETTLE
from pick_place.rod_pick import (RodPick, SHAPE, DOWN, OPEN, CLOSED,
                                 HOME, ABOVE_PICK)

# ---- Tilt settings (change these) -----------------------------------------
TILT_DEG = -45.0     # tilt angle; use -45.0 to tilt the other end down
HOLD = 2.0          # seconds to hold the tilted position

# Gripper pointing straight forward, object's middle 25 cm above the board,
# same spot in front of the arm as the pick (calculated from the arm model).
# For a 41 cm object, the low end stays ~10 cm above the board when tilted.
FORWARD = [0.000, 0.873, -0.228, 0.000, -0.558, 1.57]
TILTED = FORWARD[:5] + [FORWARD[5] + math.radians(TILT_DEG)]
# ---------------------------------------------------------------------------


class RodTilt(RodPick):
    def hold(self):
        time.sleep(HOLD)
        return True

    def run(self):
        self.get_logger().info(f'Waiting for MoveIt ... (object: {SHAPE})')
        self.move_client.wait_for_server()
        self.gripper_pose()
        self.add_box('table', size=(1.2, 1.2, 0.04), position=(0.2, 0.0, -0.025))
        self.detach_rod()
        self.add_rod()

        steps = [
            # --- pick (same as rod_pick) ---
            ('Going home',                         lambda: self.arm_to(HOME)),
            ('Opening gripper',                    lambda: self.gripper_to(OPEN)),
            ('Above object, gripper turned 90°',   lambda: self.arm_to(ABOVE_PICK)),
            ('Straight down to object',            lambda: self.move_straight(dz=-DOWN)),
            ('Attach object in MoveIt',            self.attach_rod),
            ('Grasping',                           lambda: self.gripper_to(CLOSED)),
            ('Straight up',                        lambda: self.move_straight(dz=+DOWN)),
            # --- tilt ---
            ('Turning gripper forward, lifting',   lambda: self.arm_to(FORWARD)),
            (f'Tilting {TILT_DEG:.0f}°',           lambda: self.arm_to(TILTED)),
            ('Holding',                            self.hold),
            ('Tilting back',                       lambda: self.arm_to(FORWARD)),
            # --- put back ---
            ('Back above the pick spot',           lambda: self.arm_to(ABOVE_PICK)),
            ('Putting it down',                    lambda: self.move_straight(dz=-DOWN)),
            ('Releasing',                          lambda: self.gripper_to(OPEN)),
            ('Detach object in MoveIt',            self.detach_rod),
            ('Straight up',                        lambda: self.move_straight(dz=+DOWN)),
            ('Back home',                          lambda: self.arm_to(HOME)),
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
    node = RodTilt()
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
