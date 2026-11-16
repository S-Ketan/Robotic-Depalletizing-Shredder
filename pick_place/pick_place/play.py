"""Move the PiPER (simulation or real arm) step by step by typing commands.

On the real arm it starts from the arm's CURRENT position and sends nothing
until you type your first command.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Our arm's joint 6 hits a hard stop at about 0.52 rad, and "gripper straight"
# reads about 0.63. So joint 6 is kept at 0.6 or above.
J6_STRAIGHT = 0.63
LIMITS = [(-2.62, 2.62), (0.0, 3.14), (-2.97, 0.0),
          (-1.75, 1.75), (-1.22, 1.22), (0.6, 2.97)]

HELP = """
Commands:
  2 0.1    -> turn joint 2 by +0.1 rad (use a minus sign for the other way)
  g 0.05   -> set gripper opening to 0.05 m (0 = closed, 0.07 = open)
  set 0 1.6 -1.2 0 1.2 0.63  -> jump straight to these 6 joint values
  home     -> joints 1-5 to 0, gripper straight (joint 6 = 0.63)
  q        -> quit
"""


class Play(Node):
    def __init__(self):
        super().__init__('play')
        self.pub = self.create_publisher(JointState, '/control/joint_states', 10)
        self.joints = [0.0] * 5 + [J6_STRAIGHT]
        self.gripper = 0.07
        self.active = False                    # send nothing until the first command
        self.feedback = None
        self.create_subscription(JointState, '/feedback/joint_states', self.on_feedback, 10)
        self.create_timer(0.1, self.publish)   # send the pose 10x per second
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def on_feedback(self, msg):
        self.feedback = dict(zip(msg.name, msg.position))

    def start_from_real_arm(self):
        """If a real arm is connected, start from where it is right now."""
        for _ in range(20):                    # wait up to 2 s for feedback
            if self.feedback:
                self.joints = [self.feedback.get(j, 0.0) for j in JOINTS]
                self.gripper = self.feedback.get('gripper', self.gripper)
                print('Real arm found - starting from its current position.')
                return
            time.sleep(0.1)
        print('No real arm found - starting from the home pose (simulation).')

    def publish(self):
        if not self.active:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + ['gripper']
        msg.position = self.joints + [self.gripper]
        self.pub.publish(msg)

    def gripper_position(self):
        for link in ('gripper_base', 'link6'):
            try:
                t = self.tf.lookup_transform('base_link', link, Time()).transform.translation
                return f'{link}: x={t.x * 100:.1f} cm  y={t.y * 100:.1f} cm  z={t.z * 100:.1f} cm'
            except Exception:
                pass
        return '(position not available yet)'

    def show(self):
        time.sleep(0.3)   # wait until RViz has moved the arm
        for i, v in enumerate(self.joints):
            print(f'  joint{i + 1}: {v:6.2f} rad  ({math.degrees(v):7.1f} deg)')
        print(f'  gripper: {self.gripper:.3f} m')
        print('  ' + self.gripper_position())
        print('  Copy-paste:  POSE = [' + ', '.join(f'{v:.2f}' for v in self.joints) + ']')


def main():
    rclpy.init()
    node = Play()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.start_from_real_arm()
    print(HELP)
    while True:
        node.show()
        cmd = input('\n> ').strip().split()
        if not cmd:
            continue
        if cmd[0] == 'q':
            break
        try:
            numbers = [float(c) for c in cmd[1:]]
        except ValueError:
            print(HELP)
            continue
        node.active = True
        if cmd[0] == 'set' and len(numbers) == 6:
            node.joints = [min(max(v, low), high) for v, (low, high) in zip(numbers, LIMITS)]
        elif cmd[0] == 'home':
            node.joints = [0.0] * 5 + [J6_STRAIGHT]
        elif cmd[0] == 'g' and len(cmd) == 2:
            node.gripper = min(max(float(cmd[1]), 0.0), 0.1)
        elif cmd[0] in ('1', '2', '3', '4', '5', '6') and len(cmd) == 2:
            i = int(cmd[0]) - 1
            low, high = LIMITS[i]
            new = node.joints[i] + float(cmd[1])
            if not low <= new <= high:
                print(f'  ! joint{i + 1} must stay between {low} and {high}')
            node.joints[i] = min(max(new, low), high)
        else:
            print(HELP)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
