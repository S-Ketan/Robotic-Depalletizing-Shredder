# Test Log – PiPER Shredder Project

Applied Robotics, WS 2026/27 · AgileX PiPER with AgileX gripper

**Legend:** `[SIM]` simulation · `[REAL]` real arm
**Status:** ✅ worked · ⚠️ partly · ❌ failed · ⏳ not tested yet

> Fill in the remaining `<...>` placeholders.

---

## Environment

| Item | Version / setting |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS | ROS 2 Humble (`ros-humble-desktop`) |
| Motion planning | MoveIt 2 (`ros-humble-moveit`) |
| Arm driver | `agx_arm_ros`, `ros2` branch |
| Arm SDK | `pyAgxArm` |
| Arm firmware | S-V1.6-5 (older than 1.8.5: teaching mode is not left automatically) |
| Own code | `pick_place` package: `play.py`, `sequence.py`, `sequence_moveit.py`, `rod_pick.py`; tools: `arm_check.py`, `gripper_reset.py`, `set_joint_zero.py`, `j6_range_test.py` |
| Real-arm connection | USB-CAN adapter (Geschwister Schneider / gs_usb), `can0`, 1 Mbit/s |
| Real-arm speed | `speed_percent = 20` (driver), `SPEED = 0.1` and `SETTLE` pause (MoveIt script) |

---

## Setup – known issues (for all team members)

Common problems when installing or starting the software. Not test sessions, just a checklist.

| Problem | Cause | Fix |
|---|---|---|
| `dpkg returned an error code (1)` when installing MoveIt | `sudo apt install ros-humble-moveit*` (wildcard) also pulls debug and conflicting packages | install only `ros-humble-moveit`; repair with `sudo dpkg --configure -a` |
| `ModuleNotFoundError: No module named 'catkin_pkg'` during `colcon build` | Anaconda's Python is used instead of the system Python | `conda config --set auto_activate_base false`, open a new terminal, clean rebuild |
| MoveIt crashes or reads numbers wrong | German number format (`0,5` instead of `0.5`) | `export LC_NUMERIC=en_US.UTF-8` (add to `~/.bashrc`) |
| `Package 'pick_place' not found` | workspace not sourced, or package not in `~/ros2_ws/src` | `source ~/ros2_ws/install/setup.bash`; check the package location |
| `PackageNotFoundError: No package metadata was found` | stale build files | `rm -rf build/pick_place install/pick_place`, rebuild, source |
| Script runs but the arm in RViz doesn't move | joint sliders (`control:=true`) or MoveIt send their own poses | use `display.launch.py ... control:=false` for `play`/`sequence`; MoveIt launch only for `sequence_moveit` |
| `ros2 node list` shows the driver, but `ros2 topic echo`/`hz` receive nothing (even talker/listener fail) | ROS programs can't exchange data: leftover shared-memory files after crashes, or terminals with different shells/settings | close all ROS programs, `ros2 daemon stop`, `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`, use the same shell (bash) in all terminals |

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
- Test object: the (empty) camera box provided for the later camera integration

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
| 9 | `sequence` with object (empty camera box), 3 runs | object picked and dropped | ⚠️ | 2/3 successful; in 1 run the box was not placed in the middle and a finger hit it (P7) |
| 10 | `sequence_moveit` on real arm | full sequence | ❌ | not run this day because of the joint 6 fault (P3); done successfully in Session 3 |

### Problems
**P1 – Arm ignored all movement commands**
- Symptom: `play` and driver connected (checked with `ros2 topic info /control/joint_states -v`), arm enabled, no movement
- Diagnosis: `/feedback/arm_status` showed `ctrl_mode: 2` = teaching mode
- Fix: restart arm, teach-button light off → `ctrl_mode: 1` (CAN control)

**P2 – `ros2 topic pub --once` command sometimes lost**
- Cause: message sent before the driver had discovered the publisher
- Fix: `ros2 topic pub -t 5 -r 10 ...` (send several times)

**P3 – Joint 6 fault (gone on 23.11.2026, see Session 3)**
- Symptoms:
  - joint 6 could not reach 0 rad; target "all zeros" never reached (`motion_status: 1`), and further commands were then ignored
  - by hand: hard stop at ~0.52 rad, later ~0.44 rad; "gripper straight" reads ~0.63 rad
  - effort stuck at ~−8.8 even with the joint disabled; reported temperature stuck at 53–54 °C while the joint was physically cool (others 29–41 °C)
  - enabling the arm repeatedly failed (`Failed to enable Agx_arm`)
  - joint 6 drifted back toward its stop between runs (0.74 → 0.45 rad)
  - per-joint status (`tools/arm_check.py`): `driver_overcurrent`, `driver_error_status`, `stall_status`, later also `collision_status`
  - fault persists after controller reset and after a full power cycle (motor at 36–37 °C); joints 1–5 normal
- Workarounds in code: joint 6 kept ≥ 0.6 rad (`play.py`, `sequence.py`); MoveIt path constraint keeps joint 6 in [0.6, 2.97] rad (`sequence_moveit.py`)
- Action: reported to lab staff; no repair was done
- Outcome (Session 3): the joint 6 errors were gone by themselves at the next session; the zero offset was then fixed by recalibration, full range verified, workarounds removed

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

**P7 – Finger hit the object (one run)**
- Symptom: the gripper came down on the edge of the camera box and pushed a hole into the cardboard
- Cause: box not placed in the middle under the gripper. The script goes to a fixed position and does not detect where the object is.
- Prevention: mark the pick position on the board (tape) and centre the object on the mark before each run

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
- Without object detection, placement accuracy decides success: a few cm off and the fingers hit the object. A mark on the board helps; a camera would remove this limitation (later extension).

### Sim vs. real
| Aspect | Simulation | Real arm |
|---|---|---|
| Joint 6 range | reaches 0 rad | hard stop at ~0.44–0.52 rad |
| "Gripper straight" | joint 6 = 0 | joint 6 ≈ 0.63 rad |
| Gripper closing on object | no contact model, always reaches target | stalls → overcurrent at 0 m |
| Fingertip height at AT_PICK | ~4 cm above table | ~4 cm above table (measured 23.11.2026) |

### Next steps
- Check joint 6 at the next session
- Real arm: `sequence_moveit` (straight-line moves), 3 successful pick-and-place runs in a row
- Measure fingertip heights on the real table and adjust poses
- Phase 3: elongated objects / rod, grip points, 45° tilt (prepare in simulation)

---

## Session 3 – 23.11.2026 – [REAL] Joint 6 recalibration, first MoveIt runs on the real arm

### Setup
- Professor present for the joint 6 check
- Driver: `start_single_agx_arm_rviz.launch.py` (`play`), `start_single_agx_arm_moveit.launch.py` (`sequence_moveit`), `speed_percent = 20`
- Arm firmware: S-V1.6-5
- Test object for `sequence_moveit`: the (empty) camera box provided for the later camera integration

### Tests
| # | Test | Expected | Result | Notes |
|---|------|----------|--------|-------|
| 1 | Enable arm after start | `success=True` | ❌ → ✅ | arm in teaching mode again (P1) |
| 2 | Joint status after leaving teaching mode | all joints `problems=none` | ❌ → ✅ | all joints `voltage_too_low` (P2) |
| 3 | First moves with `play` | joints respond | ⚠️ | worked at first; stopped again when joint 6 was moved (P3) |
| 4 | Joint 6 zero-point check | gripper straight = 0 rad | ❌ → ✅ | professor noticed joint 6 not aligned at 0 → recalibrated (P3) |
| 5 | Joint 6 range test (`j6_range_test.py`): +0.3 / −0.3 / 0 rad | all targets reached, no errors | ✅ | +0.300 / −0.300 / 0.000 rad, `problems: none` |
| 6 | Remove joint 6 workarounds from all scripts | "straight" = 0, full range | ✅ | see code changes |
| 7 | Feedback topics (`/feedback/joint_states`) | data at ~200 Hz | ❌ → ✅ | no data in any terminal (P4) |
| 8 | Small moves with `play`, incl. joint 6 and gripper | all respond | ✅ | |
| 9 | `sequence_moveit` on real arm, first run | full sequence | ❌ | MoveIt error −4 at "Straight down" (P5) |
| 10 | `sequence_moveit` with `SETTLE` pause, no object | full sequence | ✅ | first full MoveIt run on the real arm |
| 11 | `sequence_moveit` with object (empty camera box), 3 runs in a row | object picked and dropped each time | ✅ | 3/3 successful, video recorded (<file name / link>) |

**Milestone:** repeatable pick-and-place on the real arm with MoveIt straight-line moves (Phase 2 done).

### Problems
**P1 – Teaching mode again (`ctrl_mode: 2`)**
- Symptom: `Failed to enable Agx_arm`
- Cause: teach button had been pressed. On the PiPER the button only starts/stops a teaching recording; it does **not** switch back to laptop control. With firmware older than 1.8.5 (ours: 1.6) the arm also does not switch back automatically.
- Fix: restart the arm without touching the teach button / `exit_teach_mode` service → `ctrl_mode: CAN_CTRL`

**P2 – All joints `voltage_too_low` + `driver_error_status`**
- Cause: motor power supply problem (all joints at once)
- Fix: the power cable was not plugged in properly → plugged in firmly, then error reset → all joints `problems=none`

**P3 – Joint 6 not at zero (follow-up of Session 2, P3)**
- Note: no repair had been done since Session 2; at the start of this session joint 6 showed no errors anymore
- Symptom: arm worked at first, but stopped again when joint 6 was controlled
- Diagnosis: professor noticed that joint 6 was not aligned at 0 (gripper "straight" read ~0.63 rad instead of 0)
- Fix: joint 6 zero point recalibrated (`tools/set_joint_zero.py 6`, gripper held in its zero position, then `calibrate_joint`)
- Check: gripper straight now reads 0.0 rad; range test ±0.3 rad without errors (test 5)

**P4 – No data on any ROS topic**
- Symptom: `ros2 node list` showed the driver, but `ros2 topic echo` / `hz` received nothing; even `talker`/`listener` failed
- Cause: ROS communication between terminals broken (leftover shared-memory files after crashes and/or different shell settings)
- Fix: <which fix worked: ros2 daemon stop + removing /dev/shm/fastrtps_* / same shell in all terminals / ROS_LOCALHOST_ONLY=1 / firewall>

**P5 – MoveIt error −4 (`CONTROL_FAILED`) at "Straight down"**
- Log: `Invalid Trajectory: start point deviates from current robot state more than 0.01 – joint2 expected 1.48878, current 1.50531`
- Cause: ROS reported "Above object" as finished while the real arm (limited to 20 % firmware speed) was still moving; the next straight-line move started from a position the arm had not reached yet
- Fix: `SETTLE` pause after every step in `sequence_moveit.py`: 3.0 s worked, reduced to 1.5 s (tested, works); 1.5 s felt too slow, so the repo now has 1.0 s (**not tested yet**)
- Possible improvement: higher `speed_percent` so the firmware keeps up with MoveIt, or wait until the joints have stopped instead of a fixed pause

### Code changes from this session
- Joint 6 workarounds removed: `play.py` (`J6_STRAIGHT = 0.0`, full joint 6 range), `sequence.py` and `sequence_moveit.py` (joint 6 = 0 in all poses), joint 6 path constraint switched off (`protect_j6=False`)
- `sequence_moveit.py`: `SETTLE` pause after each step (1.5 s tested; repo currently 1.0 s, untested)
- New tools: `tools/set_joint_zero.py` (joint zero calibration), `tools/j6_range_test.py`
- New script (not tested yet): `rod_pick.py` for Phase 3 – picks an elongated object lying sideways, gripper turned 90° (joint 6 = 1.57), object attached in MoveIt while carried

### Learnings
- After every start: check `ctrl_mode` (must be 1, not 2) and `python3 tools/arm_check.py` (all joints `problems=none`).
- All joints reporting the same error at once points to a common cause (power supply), not to the joints.
- A wrong joint zero point makes the real arm and the model disagree; compare RViz with the real arm to spot it.
- On the real arm, wait until a move has finished before planning the next one (`SETTLE`).
- If ROS topics show up but deliver no data, test with `talker`/`listener` first: that separates ROS problems from arm problems.

### Sim vs. real
| Aspect | Simulation | Real arm |
|---|---|---|
| Joint 6 | 0 = gripper straight | 0 = gripper straight (after recalibration) |
| Move timing | arm is always exactly on time | arm lags behind MoveIt's timing → `SETTLE` pause needed |
| Gripper position at bottom of "Straight down" | fingertips ~4 cm above table, ~30.6 cm in front of the base centre | fingertips ~4 cm above table, ~33 cm from the back of the base (measured) |

### Next steps
- Stand-in for the rod: aluminium spirit level; measure length, height, thickness, weight
- Then `rod_pick.py` on the real arm: grip in the middle, not on the bubble windows; clear the swing area
- Clarify real rod and shredder opening dimensions; build an opening mock-up (Phase 4)
- Start the paper outline (writing starts KW 50)

---

## Session 4 – 24.11.2026 – [SIM] Phase 3: elongated object (simulation only)

### Setup
- Simulation: MoveIt `demo.launch.py` (`piper_moveit`) + `ros2 run pick_place rod_pick`
- Only tested in simulation; not yet run on the real arm

### Tests
| # | Test | Expected | Result | Notes |
|---|------|----------|--------|-------|
| 1 | Rod added to MoveIt, lying sideways in front of the arm | rod visible on the table | ✅ | |
| 2 | Gripper turned 90° (joint 6 = 1.57) above the rod | fingers on both sides of the rod | ✅ | fingers close across the rod |
| 3 | Straight down, attach rod in MoveIt, grasp | rod moves with the gripper | ✅ | |
| 4 | Carry to drop spot, lower, release, detach, home | rod stays at drop spot, "Done!" | ✅ | full `rod_pick` sequence works in simulation |

### Learnings
- With joint 6 = 0 the fingers close along y; with joint 6 = 1.57 they close along x (across a rod lying along y).
- Attaching the object in MoveIt makes MoveIt check the carried object for collisions too, important for long objects and later for the opening.

### Next steps
- Real arm: test `sequence_moveit` with `SETTLE = 1.0` s
- Stand-in for the rod on the real arm: aluminium spirit level; measure length, height, thickness, weight and set them in `rod_pick.py`
- `rod_pick.py` on the real arm: grip in the middle, not on the bubble windows; clear the swing area
- Clarify real rod and shredder opening dimensions; build an opening mock-up (Phase 4)
- Start the paper outline (writing starts KW 50)
