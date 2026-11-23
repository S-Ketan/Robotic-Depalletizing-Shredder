"""Set ONE joint's zero point to its current position (permanent!).

Usage: python3 set_joint_zero.py 6      (joint number 1-6)
Stop the ROS driver first. Support the arm. Use only with lab approval.
"""
import math
import select
import sys
import time

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

JOINT = int(sys.argv[1]) if len(sys.argv) > 1 else 6
if JOINT not in range(1, 7):
    sys.exit("Joint number must be 1-6")

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT,
                            interface="socketcan", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
time.sleep(1.0)


def angle():
    js = robot.get_joint_angles()
    return js.msg[JOINT - 1] if js is not None else float("nan")


print(f"Joint {JOINT} currently reads {angle():+.3f} rad")
if input(f"Disable joint {JOINT} so you can turn it by hand? SUPPORT THE ARM! [y/N] ").lower() != "y":
    sys.exit("Cancelled, nothing changed.")

robot.disable(JOINT)
print("Turn the joint to its zero position. Live reading below.")
print("Press Enter when it is aligned, or type q + Enter to cancel.")
while True:
    a = angle()
    print(f"\r  joint {JOINT}: {a:+.3f} rad ({math.degrees(a):+6.1f} deg)   ", end="", flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], 0.2)
    if ready:
        answer = sys.stdin.readline().strip()
        break
print()

if answer == "q":
    robot.enable(JOINT)
    sys.exit("Cancelled, nothing changed.")

if robot.calibrate_joint(JOINT):
    print("Zero point set.")
else:
    print("FAILED - nothing changed.")
time.sleep(0.5)
robot.enable(JOINT)
print(f"Joint {JOINT} now reads {angle():+.3f} rad (should be about 0)")
