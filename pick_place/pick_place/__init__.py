import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Poses in radians (joint1..joint6). Replace with YOUR values later.
HOME       = [0.0, 0.0,  0.0, 0.0, 0.0, 0.0]
ABOVE_PICK = [0.0, 1.0, -1.0, 0.0, 0.6, 0.0]
AT_PICK    = [0.0, 1.3, -1.0, 0.0, 0.3, 0.0]
ABOVE_DROP = [1.2, 1.0, -1.0, 0.0, 0.6, 0.0]

OPEN, CLOSED = 0.07, 0.0   # gripper width in meters


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.pub = self.create_publisher(JointState, '/control/joint_states', 10)
        self.current = list(HOME)
        self.gripper = OPEN

    def send(self, joints, gripper):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + ['gripper']
        msg.position = list(joints) + [gripper]
        self.pub.publish(msg)

    def move_to(self, target, seconds=2.0):
        steps = int(seconds * 50)
        start = self.current
        for i in range(1, steps + 1):
            t = i / steps
            pose = [s + (g - s) * t for s, g in zip(start, target)]
            self.send(pose, self.gripper)
            time.sleep(1 / 50)
        self.current = list(target)

    def set_gripper(self, width):
        self.gripper = width
        for _ in range(25):
            self.send(self.current, width)
            time.sleep(1 / 50)

    def run(self):
        steps = [
            ('Going home',      lambda: self.move_to(HOME)),
            ('Opening gripper', lambda: self.set_gripper(OPEN)),
            ('Above object',    lambda: self.move_to(ABOVE_PICK)),
            ('Down to object',  lambda: self.move_to(AT_PICK, 1.5)),
            ('Grasping',        lambda: self.set_gripper(CLOSED)),
            ('Lifting',         lambda: self.move_to(ABOVE_PICK, 1.5)),
            ('Moving to drop',  lambda: self.move_to(ABOVE_DROP, 3.0)),
            ('Releasing',       lambda: self.set_gripper(OPEN)),
            ('Back home',       lambda: self.move_to(HOME, 3.0)),
        ]
        for name, action in steps:
            self.get_logger().info(name)
            action()
        self.get_logger().info('Done!')


def main():
    rclpy.init()
    node = PickPlace()
    time.sleep(1.0)
    node.run()
    node.destroy_node()
    rclpy.shutdown()
