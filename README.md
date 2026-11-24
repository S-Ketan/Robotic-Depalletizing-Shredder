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

## Shortcuts (aliases)

The commands below are long, so we use these shortcuts. Add them once to `~/.bashrc`:

```bash
cat >> ~/.bashrc << 'EOF'

# --- PiPER: simulation ---
alias piper='ros2 launch agx_arm_description display.launch.py arm_type:=piper effector_type:=agx_gripper'
alias piper_view='ros2 launch agx_arm_description display.launch.py arm_type:=piper effector_type:=agx_gripper control:=false'
alias piper_moveit='LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper'

# --- PiPER: real arm ---
alias piper_can='bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000'
alias piper_real='ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper speed_percent:=20'
alias piper_real_moveit='LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper speed_percent:=20'
alias piper_stop='ros2 service call /emergency_stop std_srvs/srv/Empty'
EOF
source ~/.bashrc
```

| Shortcut | What it starts | Use with |
|---|---|---|
| `piper` | arm model in RViz with joint sliders | looking at the model, finding poses by hand |
| `piper_view` | arm model in RViz **without** sliders | `play`, `sequence` (simulation) |
| `piper_moveit` | MoveIt with a simulated arm | `sequence_moveit`, `rod_pick` (simulation) |
| `piper_can` | activates CAN (after every power-on or replug) | real arm |
| `piper_real` | real-arm driver + RViz, 20 % speed | `play`, `sequence` (real arm) |
| `piper_real_moveit` | real-arm driver + MoveIt, 20 % speed | `sequence_moveit`, `rod_pick` (real arm) |
| `piper_stop` | emergency stop | anytime on the real arm |

Run only **one** launch shortcut and **one** program that moves the arm at a time.

## Repository structure

```
├── pick_place/          ROS 2 package (run with: ros2 run pick_place ...)
│   └── pick_place/
│       ├── play.py              interactive step-by-step control
│       ├── sequence.py          pick-and-place with fixed joint poses
│       ├── sequence_moveit.py   pick-and-place with MoveIt (straight-line moves)
│       └── rod_pick.py          Phase 3: pick an elongated object lying sideways
├── tools/               helper scripts for the real arm (run with: python3 tools/...)
│   ├── arm_check.py
│   ├── gripper_reset.py
│   ├── set_joint_zero.py
│   └── j6_range_test.py
└── docs/
    └── test_log.md      lab test log (results and failure cases)
```

## Programs

| Command | Description | Simulation | Real arm |
|---|---|---|---|
| `ros2 run pick_place play` | Move the arm step by step by typing commands (`2 0.1`, `g 0.05`, `set ...`, `home`, `q`). Shows joint values and gripper position. On the real arm it starts from the arm's current position. | `piper_view` | `piper_real` |
| `ros2 run pick_place sequence` | Full pick-and-place sequence using fixed joint poses. | `piper_view` | `piper_real` |
| `ros2 run pick_place sequence_moveit` | Pick-and-place planned by MoveIt: straight-line moves down/up at the object, table as collision object, `SETTLE` pause between steps. | `piper_moveit` | `piper_real_moveit` |
| `ros2 run pick_place rod_pick` | Picks an elongated object lying sideways: gripper turned 90° (joint 6 = 1.57), object attached in MoveIt while carried. Tested in simulation only so far. | `piper_moveit` | `piper_real_moveit` |

### Main settings (top of each script)
| Setting | Meaning |
|---|---|
| `ABOVE_PICK`, `ABOVE_DROP`, `HOME` | joint poses in radians (joint 1 … joint 6) |
| `DOWN` | how far the gripper moves straight down to the object (m) |
| `OPEN`, `CLOSED` | gripper width (m); `CLOSED` = object width minus a few mm |
| `SPEED` | MoveIt speed factor (0.1 = 10 %) |
| `SETTLE` | pause after each step so the real arm finishes its move (1.5 s tested; 1.0 s not tested yet) |

## Real arm: start-up order

1. Arm in its zero pose, nothing in its reach. **Don't press the teach button** (between J5 and J6).
2. Plug the power cable **firmly** into the arm, switch it on, plug in the USB-CAN adapter.
3. `piper_can`
4. `piper_real` (or `piper_real_moveit`). Wait until it prints `Agx_arm feedback is ready, control is now enabled`.
5. Check the arm:
   ```bash
   python3 tools/arm_check.py
   ```
   `ctrl_mode` must be `CAN_CTRL` (1) or `STANDBY` (0), and all joints `enabled=True`, `problems=none`.
6. Keep `piper_stop` typed in a spare terminal.
7. Run the program. For pick-and-place, place the object **centred on the marked pick spot**.

## Tools (real arm only)

These scripts talk to the arm directly over CAN using the pyAgxArm SDK. Run them with plain `python3` (no conda), with CAN active.

| Command | Description |
|---|---|
| `python3 tools/arm_check.py` | Shows arm status and, for each joint: enabled, motor temperature, current and active errors (overcurrent, stall, collision, low voltage, …). Does not move the arm. |
| `python3 tools/arm_check.py --reset` | Clears a protective stop and re-enables the joints. ⚠️ Support the arm, it may drop briefly. |
| `python3 tools/gripper_reset.py` | Clears a gripper error (e.g. overcurrent) and opens the gripper fully. |
| `python3 tools/j6_range_test.py` | Moves joint 6 to +0.3, −0.3 and 0 rad at 20 % speed and reports position and errors. Stop the ROS driver first. |
| `python3 tools/set_joint_zero.py <joint>` | Sets a joint's **current** position as its new zero point (**permanent**). Only with the supervisor's approval; stop the ROS driver first; support the arm. |

### Common status values (`/feedback/arm_status`)
| Field | Value | Meaning |
|---|---|---|
| `ctrl_mode` | 0 | Standby (normal after start/reset) |
| `ctrl_mode` | 1 | CAN control (OK) |
| `ctrl_mode` | 2 | Teaching mode: arm ignores commands (see below) |
| `arm_status` | 0 | Normal |
| `motion_status` | 1 | Last target not reached (a joint is blocked) |

## Troubleshooting

| Problem | Fix |
|---|---|
| `ctrl_mode: 2` (teaching mode), enabling fails | The teach button only starts/stops a recording, it doesn't switch back. Our firmware (1.6) also doesn't switch back by itself: restart the arm without touching the button, or `ros2 service call /exit_teach_mode std_srvs/srv/Empty` (arm moves home and may drop, hold it). |
| All joints `voltage_too_low` | Power cable not plugged in properly. Plug it in firmly, then `python3 tools/arm_check.py --reset`. |
| Driver dies with `Failed to get firmware version` | No CAN data: check power and CAN wires, `piper_can`, `candump can0` must show data. |
| `candump can0` → `No such device` | USB-CAN adapter not detected: replug it (other port), check `lsusb`; restart the laptop if needed. |
| Topics are listed but `ros2 topic echo`/`hz` get nothing (even `talker`/`listener` fail) | Close all ROS programs, `ros2 daemon stop`, `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`, use the same shell (bash) in all terminals. |
| MoveIt error `-4` / `start point deviates from current robot state` | The real arm was still moving. Increase `SETTLE` in the script. |
| Gripper stops responding after closing on an object | Overcurrent: `python3 tools/gripper_reset.py`; set `CLOSED` larger. |

## Safety notes

- **Gripper:** never close to 0 m with an object inside (overcurrent shutdown). Set `CLOSED` to the object width minus a few mm.
- **Object placement:** the scripts go to fixed positions and don't detect the object. Mark the pick spot on the board and centre the object on it, otherwise the fingers can hit it.
- **Long objects:** clear the swing area before the arm turns to the drop spot.
- Always test new code in **simulation first**, then on the real arm at low speed with `piper_stop` ready.

More details, results and failure cases: [`docs/test_log.md`](docs/test_log.md).
