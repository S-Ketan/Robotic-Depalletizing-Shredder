"""Simple pick-and-place with fixed joint poses (simulation or real arm)."""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Poses in radians (joint1..joint6).
# Joint 6 = 0.63 on our arm: it has a hard stop at ~0.52 and can't reach 0.
HOME       = [0.00, 0.00,  0.00, 0.00, 0.00, 0.00]
ABOVE_PICK = [0.00, 1.60, -1.20, 0.00, 1.20, 0.00]   # x≈30, z≈25
AT_PICK    = [0.00, 1.70, -1.00, 0.00, 0.95, 0.00]   # straight below
ABOVE_DROP = [1.57, 1.60, -1.20, 0.00, 1.20, 0.00]   # turned 90° left

OPEN = 0.07      # gripper fully open (meters)
CLOSED = 0.025   # a few mm LESS than the object's width - never 0 with an object!


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.pub = self.create_publisher(JointState, '/control/joint_states', 10)
        self.current = list(HOME)
        self.gripper = OPEN
        self.feedback = None
        self.create_subscription(JointState, '/feedback/joint_states', self.on_feedback, 10)

    def on_feedback(self, msg):
        self.feedback = dict(zip(msg.name, msg.position))

    def start_from_real_arm(self):
        """On the real arm, start from where it actually is (not from HOME)."""
        for _ in range(20):                       # wait up to 2 s for feedback
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.feedback:
                self.current = [self.feedback.get(j, 0.0) for j in JOINTS]
                self.get_logger().info('Real arm found - starting from its current position')
                return
        self.get_logger().info('No real arm found - simulation')

    def send(self, joints, gripper):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + ['gripper']
        msg.position = list(joints) + [gripper]
        self.pub.publish(msg)

    def move_to(self, target, seconds=2.0):
        """Move smoothly from the current pose to target."""
        steps = int(seconds * 50)          # 50 small steps per second
        start = self.current
        for i in range(1, steps + 1):
            t = i / steps
            pose = [s + (g - s) * t for s, g in zip(start, target)]
            self.send(pose, self.gripper)
            time.sleep(1 / 50)
        self.current = list(target)

    def set_gripper(self, width):
        self.gripper = width
        for _ in range(50):                # hold the command for 1 s
            self.send(self.current, width)
            time.sleep(1 / 50)

    def run(self):
        steps = [
            ('Going home',      lambda: self.move_to(HOME, 3.0)),
            ('Opening gripper', lambda: self.set_gripper(OPEN)),
            ('Above object',    lambda: self.move_to(ABOVE_PICK, 3.0)),
            ('Down to object',  lambda: self.move_to(AT_PICK, 2.0)),
            ('Grasping',        lambda: self.set_gripper(CLOSED)),
            ('Lifting',         lambda: self.move_to(ABOVE_PICK, 2.0)),
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
    node.start_from_real_arm()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
