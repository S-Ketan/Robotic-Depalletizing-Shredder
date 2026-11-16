# Test Log – PiPER Shredder Project

Applied Robotics, WS 2026/27 · AgileX PiPER with AgileX gripper

**Legend:** `[SIM]` simulation · `[REAL]` real arm
**Status:** ✅ worked · ⚠️ partly · ❌ failed · ⏳ not tested yet

> Fill in the `<...>` placeholders (names, test object, number of runs).

---

## Environment

| Item | Version / setting |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS | ROS 2 Humble (`ros-humble-desktop`) |
| Motion planning | MoveIt 2 (`ros-humble-moveit`) |
| Arm driver | `agx_arm_ros`, `ros2` branch |
| Arm SDK | `pyAgxArm` |
| Own code | `pick_place` package: `play.py`, `sequence.py`, `sequence_moveit.py`; tools: `arm_check.py`, `gripper_reset.py` |
| Real-arm connection | USB-CAN adapter (Geschwister Schneider / gs_usb), `can0`, 1 Mbit/s |
| Real-arm speed | `speed_percent = 20` (driver), `SPEED = 0.1` (MoveIt script) |

---

## Setup – known issues (for all team members)

Common problems when installing the software. Not test sessions, just a checklist for new setups.

| Problem | Cause | Fix |
|---|---|---|
| `dpkg returned an error code (1)` when installing MoveIt | `sudo apt install ros-humble-moveit*` (wildcard) also pulls debug and conflicting packages | install only `ros-humble-moveit`; repair with `sudo dpkg --configure -a` |
| `ModuleNotFoundError: No module named 'catkin_pkg'` during `colcon build` | Anaconda's Python is used instead of the system Python | `conda config --set auto_activate_base false`, open a new terminal, clean rebuild |
| MoveIt crashes or reads numbers wrong | German number format (`0,5` instead of `0.5`) | `export LC_NUMERIC=en_US.UTF-8` (add to `~/.bashrc`) |
| `Package 'pick_place' not found` | workspace not sourced, or package not in `~/ros2_ws/src` | `source ~/ros2_ws/install/setup.bash`; check the package location |
| `PackageNotFoundError: No package metadata was found` | stale build files | `rm -rf build/pick_place install/pick_place`, rebuild, source |
| Script runs but the arm in RViz doesn't move | joint sliders (`control:=true`) or MoveIt send their own poses | use `display.launch.py ... control:=false` for `play`/`sequence`; MoveIt launch only for `sequence_moveit` |

---

## Session 1 – 11.11.2026 – [SIM] Pick-and-place poses and MoveIt straight-line version

### Setup
- Simulation: `display.launch.py` (`play`, `sequence`), MoveIt `demo.launch.py` (`sequence_moveit`)

### Tests
| # | Test | Expected | Result | Notes |
|---|------|----------|--------|-------|
| 1 | Find ABOVE_PICK with `play` | gripper pointing down, ~10 cm above table | ✅ | `[0, 1.60, -1.20, 0, 1.20, 0]` → gripper_base x = 29.8 cm, z = 25.3 cm, ~3° from vertical |
| 2 | Find AT_PICK (straight below) | same x, lower z, gripper vertical | ⚠️ → ✅ | first try `[0, 1.70, -1.00, 0, 0.82, 0]` was tilted ~8° (x = 30.5, z = 17.7); corrected joint 5 to 0.95 → x = 29.3, z = 17.6 |
| 3 | Find ABOVE_DROP (joint 1 turned 90°) | same height, rotated to the left | ✅ | `[1.57, 1.60, -1.20, 0, 1.20, 0]` → x = 0, y = 29.8, z = 25.3 |
| 4 | `sequence` with own poses | full pick-and-place in RViz | ✅ | |
| 5 | `sequence_moveit`: MoveIt planning + Cartesian straight-line down/up (`DOWN = 0.075 m`) | gripper moves vertically at the object | ✅ | confirmed working 16.11.2026 |
| 6 | Table added as collision object | table visible, no paths through it | ✅ | |
| 7 | Collision check: `DOWN = 0.20 m` (into the table) | MoveIt refuses ("Straight line only possible for X %") | ⏳ | <result> |

### Learnings
- Coordinates: x forward, y left, z up, measured from the bottom centre of the arm base (table height).
- The shown point is `gripper_base`. The fingertips are ~14 cm further out, so z must stay above ~15 cm when pointing down.
- Joint 5 limit is ±1.22 rad. ABOVE_PICK uses 1.20, very close to the limit.
- Joint-space moves give a curved gripper path. MoveIt Cartesian moves give straight lines (professor's recommendation).
- In the MoveIt simulation, joint states are on `/control/joint_states`.

---

## Session 2 – 16.11.2026 – [REAL] First real-arm session

### Setup
- Driver: `start_single_agx_arm_rviz.launch.py`, `speed_percent = 20`
- Test object: <name, size, e.g. "foam block 3 × 3 × 3 cm">

### Tests
| # | Test | Expected | Result | Notes |
|---|------|----------|--------|-------|
| 1 | CAN activation + driver start | joint feedback on `/feedback/joint_states` | ✅ | |
| 2 | Move with `play` | joints move | ❌ | arm in teaching mode (P1) |
| 3 | Enable arm | `success=True` | ✅ | |
| 4 | Single joint move via `/control/move_j` (joint 1 = 0.2 rad) | base turns ~11° | ✅ | first real movement |
| 5 | Move back (joint 1 = 0, all joints 0) | base turns back | ❌ | joint 6 could not reach 0 (P3); `--once` message also unreliable (P2) |
| 6 | Gripper open/close | gripper moves | ✅ | |
| 7 | Close gripper to 0 m with object inside | object gripped | ❌ | gripper overcurrent (P4) |
| 8 | `sequence` without object (joint 6 = 0.63) | full sequence | ✅ | |
| 9 | `sequence` with object | object picked and dropped | <✅/❌> | 5/3 |
| 10 | `sequence_moveit` on real arm | full sequence | ⏳ | not possible, joint 6 fault (P3) |

### Problems
**P1 – Arm ignored all movement commands**
- Symptom: `play` and driver connected (checked with `ros2 topic info /control/joint_states -v`), arm enabled, no movement
- Diagnosis: `/feedback/arm_status` showed `ctrl_mode: 2` = teaching mode
- Fix: restart arm, teach-button light off → `ctrl_mode: 1` (CAN control)

**P2 – `ros2 topic pub --once` command sometimes lost**
- Cause: message sent before the driver had discovered the publisher
- Fix: `ros2 topic pub -t 5 -r 10 ...` (send several times)

**P3 – Joint 6 fault (unresolved, handed to lab staff)**
- Symptoms:
  - joint 6 could not reach 0 rad; target "all zeros" never reached (`motion_status: 1`), and further commands were then ignored
  - by hand: hard stop at ~0.52 rad, later ~0.44 rad; "gripper straight" reads ~0.63 rad
  - effort stuck at ~−8.8 even with the joint disabled; reported temperature stuck at 53–54 °C while the joint was physically cool (others 29–41 °C)
  - enabling the arm repeatedly failed (`Failed to enable Agx_arm`)
  - joint 6 drifted back toward its stop between runs (0.74 → 0.45 rad)
  - per-joint status (`tools/arm_check.py`): `driver_overcurrent`, `driver_error_status`, `stall_status`, later also `collision_status`
  - fault persists after controller reset and after a full power cycle (motor at 36–37 °C); joints 1–5 normal
- Workarounds in code: joint 6 kept ≥ 0.6 rad (`play.py`, `sequence.py`); MoveIt path constraint keeps joint 6 in [0.6, 2.97] rad (`sequence_moveit.py`)
- Action: arm handed to lab staff (check gripper cable routing, gripper mounting, joint 6 mechanics / calibration)

**P4 – Gripper stopped responding**
- Symptom: after closing to 0 m on an object, no reaction to commands
- Diagnosis: `/feedback/gripper_status`: `driver_overcurrent: true`, `driver_error_status: true`, `driver_enable_status: false`
- Fix: `tools/gripper_reset.py` (reset + open)
- Prevention: `CLOSED` = object width minus a few mm (now 0.025 m)

**P5 – CAN adapter not detected after replugging**
- Symptom: `candump can0` → `No such device`; `lsusb` showed only the root hubs
- Fix: laptop restart; adapter then detected again (`Geschwister Schneider CAN adapter`), `can_activate.sh`

**P6 – Driver crashed on start**
- Messages: `Timed out waiting for low-speed feedback`, `Failed to auto-enable the arm`, `Failed to get firmware version`, process died
- Cause: no CAN data from the arm (adapter not detected / CAN not activated after power cycle)
- Fix: see P5, then `can_activate.sh` → `candump can0` shows data → start driver

### Code changes from this session
- `play.py`: starts from the real arm's current position, sends nothing until the first command, joint 6 limited to ≥ 0.6 rad, `home` sends joint 6 to 0.63
- `sequence.py`: joint 6 = 0.63 in all poses, `CLOSED = 0.025`, starts from the real arm's position, slower moves
- `sequence_moveit.py`: joint 6 = 0.63, joint 6 path constraint, `CLOSED = 0.025`, `SPEED = 0.1`, gripper step only warns if the exact width isn't reached
- New tools: `tools/arm_check.py` (per-joint status, `--reset`), `tools/gripper_reset.py`

### Learnings
- Check `/feedback/arm_status` first: `ctrl_mode` must be 1, `arm_status` 0, `motion_status` 0.
- Start-up order: power on → `can_activate.sh` → `candump can0` shows data → driver → `arm_check.py` → scripts.
- A joint that can't reach its target blocks all further moves (`motion_status: 1`).
- Scripts must start from the real arm's current position, never from assumed zeros.
- Never close the gripper to 0 m with an object inside.

### Sim vs. real
| Aspect | Simulation | Real arm |
|---|---|---|
| Joint 6 range | reaches 0 rad | hard stop at ~0.44–0.52 rad |
| "Gripper straight" | joint 6 = 0 | joint 6 ≈ 0.63 rad |
| Gripper closing on object | no contact model, always reaches target | stalls → overcurrent at 0 m |
| Fingertip height at AT_PICK | ~4 cm above table | ~ yet to measure..|

### Next steps
- Joint 6 repair (lab staff)
- Real arm: `sequence_moveit` (straight-line moves), 3 successful pick-and-place runs in a row
- Measure fingertip heights on the real table and adjust poses
- Phase 3: elongated objects / rod, grip points, 45° tilt (prepare in simulation)
