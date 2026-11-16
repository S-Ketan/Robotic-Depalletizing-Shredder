"""Show each joint's status. Use --reset to clear a protective stop and re-enable."""
import sys
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT,
                            interface="socketcan", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
time.sleep(1.0)

FLAGS = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
         "driver_overheating", "collision_status", "driver_error_status", "stall_status")


def show():
    st = robot.get_arm_status()
    print("Arm status:", st.msg if st is not None else "no data")
    for j in range(1, 7):
        ds = robot.get_driver_states(j)
        if ds is None:
            print(f"  joint{j}: NO DATA")
            continue
        m, f = ds.msg, ds.msg.foc_status
        problems = [n for n in FLAGS if getattr(f, n)]
        print(f"  joint{j}: enabled={f.driver_enable_status}  motor={m.motor_temp}°C  "
              f"current={m.bus_current}A  problems={problems or 'none'}")


show()
if "--reset" in sys.argv:
    print("\nResetting and re-enabling ...")
    robot.reset()
    time.sleep(0.5)
    robot.enable()
    time.sleep(1.0)
    show()
