"""Clear a gripper error and open the gripper (uses the AgileX SDK directly)."""
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT,
                            interface="socketcan", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
time.sleep(1.0)

print("Before:", gripper.get_gripper_status())
gripper.reset_gripper()          # clear the error
time.sleep(0.5)
gripper.move_gripper(width=0.07, force=1.0)   # open fully
time.sleep(2.0)
print("After:", gripper.get_gripper_status())
