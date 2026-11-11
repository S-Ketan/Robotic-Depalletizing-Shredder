# PiPER Shredder Project


Pick-and-place project using the AgileX PiPER robotic arm. The goal is to pick up objects (later a long rod) and guide them into a shredder opening.

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- AgileX PiPER driver: [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) (`ros2` branch)

## Setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/S-Ketan/Robotic-Depalletizing-Shredder.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Programs

| Command | Description |
|---|---|
| `ros2 run pick_place play` | Move the arm step by step by typing commands. |
| `ros2 run pick_place sequence` | Execute a full pick-and-place sequence using fixed poses. |

## Simulation Viewer

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=piper effector_type:=agx_gripper control:=false
```
