# PiPER Shredder Project

Pick-and-place project using the AgileX PiPER robotic arm. The goal is to pick up objects (later a long rod) and guide them into a shredder opening.

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 (`sudo apt install ros-humble-moveit`)
- AgileX PiPER driver: [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) (`ros2` branch)
- AgileX Python SDK: [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) (needed for the tools)

> ⚠️ If you use Anaconda: run `conda deactivate` before building or running anything. ROS 2 needs the system Python.

## Setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/S-Ketan/Robotic-Depalletizing-Shredder.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Repository structure

```
├── pick_place/          ROS 2 package (run with: ros2 run pick_place ...)
│   └── pick_place/
│       ├── play.py              interactive step-by-step control
│       ├── sequence.py          pick-and-place with fixed joint poses
│       └── sequence_moveit.py   pick-and-place with MoveIt (straight-line moves)
├── tools/               helper scripts for the real arm (run with: python3 tools/...)
│   ├── arm_check.py
│   └── gripper_reset.py
└── docs/
    └── test_log.md      lab test log (results and failure cases)
```

## Programs

| Command | Description |
|---|---|
| `ros2 run pick_place play` | Move the arm step by step by typing commands (`2 0.1`, `g 0.05`, `set ...`, `home`, `q`). Shows joint values and gripper position. |
| `ros2 run pick_place sequence` | Full pick-and-place sequence using fixed joint poses. |
| `ros2 run pick_place sequence_moveit` | Pick-and-place planned by MoveIt: straight-line moves down/up at the object, table as collision object. |

## Which launch file goes with which program

| Program | Simulation | Real arm |
|---|---|---|
| `play`, `sequence` | `ros2 launch agx_arm_description display.launch.py arm_type:=piper effector_type:=agx_gripper control:=false` | `ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper speed_percent:=20` |
| `sequence_moveit` | `LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper` | `LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper speed_percent:=20` |

Run only **one** program that moves the arm at a time.

## Real arm: start-up order

1. Teach-button light (between J5 and J6) must be **off**, arm in zero pose.
2. Power on the arm, plug in the USB-CAN adapter.
3. Activate CAN (after every power-on or replug):
   ```bash
   bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000
   ```
4. Start the real-arm launch file (see table above).
5. Check the arm: `python3 tools/arm_check.py`. All joints should show `enabled=True` and `problems=none`.
6. Keep the emergency stop ready:
   ```bash
   ros2 service call /emergency_stop std_srvs/srv/Empty
   ```

## Tools (real arm only)

These scripts talk to the arm directly over CAN using the pyAgxArm SDK. Run them with plain `python3` (no conda), with CAN active.

| Command | Description |
|---|---|
| `python3 tools/arm_check.py` | Shows arm status and, for each joint: enabled, motor temperature, current and active errors (overcurrent, stall, collision, overheating, …). Does not move the arm. |
| `python3 tools/arm_check.py --reset` | Clears a protective stop and re-enables the joints. ⚠️ Support the arm, it may drop briefly. |
| `python3 tools/gripper_reset.py` | Clears a gripper error (e.g. overcurrent) and opens the gripper fully. |

### Common status values (`/feedback/arm_status`)
| Field | Value | Meaning |
|---|---|---|
| `ctrl_mode` | 1 | CAN control (OK) |
| `ctrl_mode` | 2 | Teaching mode: arm ignores commands → restart arm, teach light off |
| `arm_status` | 0 | Normal |
| `motion_status` | 1 | Last target not reached (a joint is blocked) |

## Known issues / safety notes

- **Joint 6 (our arm):** hard stop at about 0.5 rad, cannot reach 0. All scripts keep joint 6 at **0.63 rad or above** ("gripper straight"). See `docs/test_log.md`.
- **Gripper:** never close to 0 m with an object inside, as this triggers an overcurrent shutdown. Set `CLOSED` to the object width minus a few mm.
- Always test new code in **simulation first**, then on the real arm at low speed.
