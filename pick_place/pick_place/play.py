"""Move the simulated PiPER step by step by typing commands."""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
LIMITS = [(-2.62, 2.62), (0.0, 3.14), (-2.97, 0.0),
          (-1.75, 1.75), (-1.22, 1.22), (-2.97, 2.97)]

HELP = """
Commands:
  2 0.1    -> turn joint 2 by +0.1 rad (use a minus sign for the other way)
  g 0.05   -> set gripper opening to 0.05 m (0 = closed, 0.07 = open)
  set 0 1.5 -1 0 1.22 0  -> jump straight to these 6 joint values
  home     -> all joints back to 0
  q        -> quit
"""


class Play(Node):
    def __init__(self):
        super().__init__('play')
        self.pub = self.create_publisher(JointState, '/control/joint_states', 10)
        self.joints = [0.0] * 6
        self.gripper = 0.07
        self.create_timer(0.1, self.publish)   # send the pose 10x per second
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def publish(self):
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
        if cmd[0] == 'set' and len(numbers) == 6:
            node.joints = [min(max(v, low), high) for v, (low, high) in zip(numbers, LIMITS)]
        elif cmd[0] == 'home':
            node.joints = [0.0] * 6
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
