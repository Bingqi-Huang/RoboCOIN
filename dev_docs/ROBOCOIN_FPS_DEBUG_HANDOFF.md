# RoboCOIN + Realman Dual-Arm Data Collection: FPS Debug & Fix Handoff

> **Purpose**: Full handoff document for a coding agent (Claude Code or similar) to run all
> diagnostic experiments and implement fixes to improve data collection FPS from current
> ~3-4Hz to ≥15Hz for pi0.5 fine-tuning.

---

## 0. Context & Goal

- **Repo in use**: [FlagOpen/RoboCOIN](https://github.com/FlagOpen/RoboCOIN) (fork of LeRobot v0.3.4)
- **Hardware**: Two Realman arms (leader + follower) on same wired LAN, one Linux PC
- **Cameras**: Intel RealSense (wrist, currently opened via OpenCV — wrong), RGB webcam (scene)
- **Target**: Collect demos to fine-tune **OpenPI pi0.5 base model**
- **Current state**: Actual collection rate is only **3–4 Hz** regardless of `dataset.fps` setting
- **Target collection rate**: ≥15 Hz (ideally 20–30 Hz)

### How to measure actual Hz (use this in every experiment)
```
actual_hz = (video_length_seconds × dataset.fps) / actual_wall_clock_time_seconds
```
Or add this to the top/bottom of the main loop in `record.py`:
```python
import time
_t0 = time.perf_counter()
# ... existing loop body ...
print(f"[LOOP] {1/(time.perf_counter()-_t0):.2f} Hz  loop_time={(time.perf_counter()-_t0)*1000:.1f}ms")
```

---

## 1. Repository Layout (Relevant Files)

```
src/lerobot/
├── record.py                                          ← main recording loop (sync)
├── robots/
│   ├── bi_realman/
│   │   ├── bi_realman.py                             ← dual-arm follower logic  ← EDIT THIS
│   │   ├── bi_realman_end_effector.py
│   │   └── configuration_bi_realman.py               ← config defaults          ← EDIT THIS
│   ├── realman/
│   │   ├── realman.py                                ← single-arm follower      ← EDIT THIS
│   │   └── configuration_realman.py
│   └── base_robot/
│       └── configuration_base_robot.py
├── teleoperators/
│   ├── bi_realman_leader/
│   │   ├── bi_realman_leader.py                      ← leader read logic        ← EDIT THIS
│   │   └── configuration_bi_realman_leader.py
│   └── realman_leader/
│       ├── realman_leader.py
│       └── configuration_realman_leader.py
└── cameras/
    ├── opencv.py
    └── intelrealsense.py                             ← should use this for wrist cam
```

---

## 2. Root Cause Analysis

The recording loop in `record.py` is a **single synchronous loop**:

```
T_frame = T_leader_read(TCP)
         + T_follower_write(TCP) + T_wait_second(sleep)
         + T_follower_read(TCP)
         + T_cam_wrist + T_cam_scene
         + T_dataset_add_frame + T_video_encode
         + T_visualization(matplotlib)
         + T_busy_wait
```

When any single term exceeds `1/fps`, `busy_wait` → 0 and the whole loop slows down.
At 3–4 Hz, `T_frame ≈ 250–330ms`. We need to find which terms sum to that.

### Known bottlenecks (confirmed or strongly suspected)

| Factor | Current State | Estimated Cost |
|--------|--------------|----------------|
| `wait_second=0.1` | Explicit sleep after every follower command | 100ms/frame |
| `rm_movej()` planning interface | Wrong interface for teleop; planning overhead | ~20–50ms |
| 4× TCP state reads per frame (2 leader + 2 follower) | Polling via TCP each frame | ~10–40ms total |
| Matplotlib visualization | `visualize=True, draw_2d=True, draw_3d=True` | ~50–200ms |
| RealSense via OpenCV | Wrong camera path (UVC, no HW timestamps) | Unknown |
| `dataset.add_frame()` blocking | Video encoding on main thread | Unknown |

---

## 3. Experiment Plan

Run experiments in this order: **D → A → B → C → E → F → G**

Start with D because it gives you raw ms numbers — all later experiments make more sense once you know the per-call costs.

For each experiment: run for **20 seconds WCT**, record video length, compute actual Hz.

---

### Group D — Instrument SDK Call Timings (DO FIRST, no lerobot-record needed)

**Goal**: Get exact millisecond cost of each SDK call.

**How**: Add timing prints to `bi_realman.py` and `bi_realman_leader.py`:

```python
# Add to bi_realman.py _get_joint_state()
import time
def _get_joint_state(self):
    t0 = time.perf_counter()
    result = self.arm.rm_get_current_arm_state()
    print(f"[FOLLOWER_GET_LEFT]  {(time.perf_counter()-t0)*1000:.2f}ms")
    return ...  # rest of existing logic

# Add to bi_realman.py _set_joint_state()
def _set_joint_state(self, joint_state):
    t0 = time.perf_counter()
    self.arm.rm_movej(joints_deg, self.config.velocity, 0, 0, int(self.config.block))
    print(f"[FOLLOWER_SET_LEFT]  {(time.perf_counter()-t0)*1000:.2f}ms")
    t1 = time.perf_counter()
    self.arm.rm_set_gripper_position(...)
    print(f"[GRIPPER_LEFT]       {(time.perf_counter()-t1)*1000:.2f}ms")
    ...

# Same pattern for right arm and for leader (bi_realman_leader.py _get_joint_state)
```

**Write a standalone timing script** (run independently, no lerobot-record needed):

```python
# save as: scripts/timing_test.py
import time
from Robotic_Arm.rm_robot_interface import *

robot_left = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle_left = robot_left.rm_create_robot_arm("169.254.128.18", 8080)

robot_right = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle_right = robot_right.rm_create_robot_arm("169.254.128.19", 8080)

print("=== Measuring SDK call latencies (100 samples each) ===\n")

# Test 1: rm_get_current_arm_state latency
times = []
for _ in range(100):
    t0 = time.perf_counter()
    robot_left.rm_get_current_arm_state()
    times.append((time.perf_counter() - t0) * 1000)
print(f"rm_get_current_arm_state (left):  mean={sum(times)/len(times):.2f}ms  max={max(times):.2f}ms  min={min(times):.2f}ms")

# Repeat for right arm
times = []
for _ in range(100):
    t0 = time.perf_counter()
    robot_right.rm_get_current_arm_state()
    times.append((time.perf_counter() - t0) * 1000)
print(f"rm_get_current_arm_state (right): mean={sum(times)/len(times):.2f}ms  max={max(times):.2f}ms  min={min(times):.2f}ms")

# Test 2: rm_movej (non-blocking) latency
# Get current joints first
state = robot_left.rm_get_current_arm_state()
current_joints = state[1]['joint']  # adjust key name as needed
times = []
for _ in range(100):
    t0 = time.perf_counter()
    robot_left.rm_movej(current_joints, 30, 0, 0, 0)  # block=0
    times.append((time.perf_counter() - t0) * 1000)
print(f"rm_movej non-blocking (left):     mean={sum(times)/len(times):.2f}ms  max={max(times):.2f}ms  min={min(times):.2f}ms")

# Test 3: rm_movej_canfd latency
times = []
for _ in range(100):
    t0 = time.perf_counter()
    robot_left.rm_movej_canfd(current_joints, follow=True, trajectory_mode=1, radio=50)
    times.append((time.perf_counter() - t0) * 1000)
print(f"rm_movej_canfd (left):            mean={sum(times)/len(times):.2f}ms  max={max(times):.2f}ms  min={min(times):.2f}ms")

# Test 4: Full dual-arm get+set cycle (simulating one teleop frame)
times = []
for _ in range(50):
    t0 = time.perf_counter()
    robot_left.rm_get_current_arm_state()   # leader left read
    robot_right.rm_get_current_arm_state()  # leader right read
    robot_left.rm_get_current_arm_state()   # follower left read
    robot_left.rm_movej(current_joints, 30, 0, 0, 0)  # follower left write
    robot_right.rm_get_current_arm_state()  # follower right read
    robot_right.rm_movej(current_joints, 30, 0, 0, 0) # follower right write
    times.append((time.perf_counter() - t0) * 1000)
print(f"\nFull teleop cycle (rm_movej):     mean={sum(times)/len(times):.2f}ms  → theoretical max {1000//(sum(times)/len(times)):.0f} Hz")

# Test 5: Same but with canfd
times = []
for _ in range(50):
    t0 = time.perf_counter()
    robot_left.rm_get_current_arm_state()
    robot_right.rm_get_current_arm_state()
    robot_left.rm_movej_canfd(current_joints, follow=True, trajectory_mode=1, radio=50)
    robot_right.rm_movej_canfd(current_joints, follow=True, trajectory_mode=1, radio=50)
    times.append((time.perf_counter() - t0) * 1000)
print(f"Full teleop cycle (rm_movej_canfd): mean={sum(times)/len(times):.2f}ms → theoretical max {1000//(sum(times)/len(times)):.0f} Hz")

robot_left.rm_delete_robot_arm()
robot_right.rm_delete_robot_arm()
```

**Expected outputs to record**:
- `rm_get_current_arm_state` latency per call (expect 5–30ms each)
- `rm_movej` non-blocking latency (expect 5–20ms)
- `rm_movej_canfd` latency (expect 1–5ms)
- Full cycle theoretical max Hz with each interface

---

### Group A — Isolate `wait_second`

**Everything else stays at baseline. Only change `wait_second`.**

```bash
# A1 - baseline
lerobot-record --robot.type=bi_realman --robot.wait_second=0.1 \
  --robot.block=False --robot.visualize=True \
  --robot.cameras="{wrist:{type:opencv,index_or_path:6,width:640,height:480,fps:30}, scene:{type:opencv,index_or_path:0,width:640,height:480,fps:30}}" \
  --robot.init_type=none --teleop.init_type=none \
  --teleop.type=bi_realman_leader \
  --teleop.ip_left=169.254.128.18 --teleop.port_left=8080 \
  --teleop.ip_right=169.254.128.19 --teleop.port_right=8080 \
  --robot.ip_left=169.254.128.18 --robot.port_left=8080 \
  --robot.ip_right=169.254.128.19 --robot.port_right=8080 \
  --dataset.repo_id=test/exp_A1 --dataset.fps=15 \
  --dataset.episode_time_s=20 --dataset.num_episodes=1 \
  --dataset.push_to_hub=False

# A2 - wait_second=0.05
# A3 - wait_second=0.01
# A4 - wait_second=0.005
# A5 - wait_second=0.0   ← MOST IMPORTANT: upper bound without explicit sleep
```

Change only the `--robot.wait_second=X` value between runs.
Also edit `configuration_bi_realman.py` default and `configuration_bi_realman_leader.py` default to match, OR pass via CLI if supported.

**Decision point**: If A5 is still ≤5 Hz → explicit sleep is NOT the only bottleneck. Proceed to C1 immediately.

---

### Group B — Isolate Visualization

**Base: use A3 settings (`wait_second=0.01`)**

```bash
# B1 - visualization fully on (matches default)
--robot.visualize=True --robot.draw_2d=True --robot.draw_3d=True

# B2 - partial off
--robot.visualize=True --robot.draw_2d=False --robot.draw_3d=False

# B3 - visualization fully off   ← RECOMMENDED from now on
--robot.visualize=False --robot.draw_2d=False --robot.draw_3d=False
```

**Decision point**: If B3 > B1 by more than 2 Hz → visualization is a meaningful bottleneck. Always use `--robot.visualize=False` from this point.

---

### Group C — Isolate Camera Paths

**Base: A5 (`wait_second=0.0`) + B3 (`visualize=False`)**

```bash
# C1 - NO cameras at all (pure robot SDK overhead)
# Remove --robot.cameras entirely
lerobot-record --robot.type=bi_realman --robot.wait_second=0.0 \
  --robot.visualize=False --robot.init_type=none \
  --teleop.type=bi_realman_leader --teleop.init_type=none \
  ... (arm IPs) ...
  --dataset.repo_id=test/exp_C1 --dataset.fps=15 \
  --dataset.episode_time_s=20 --dataset.num_episodes=1 \
  --dataset.push_to_hub=False

# C2 - scene camera only (OpenCV)
--robot.cameras="{scene:{type:opencv,index_or_path:0,width:640,height:480,fps:30}}"

# C3 - wrist camera only (current wrong path: OpenCV)
--robot.cameras="{wrist:{type:opencv,index_or_path:6,width:640,height:480,fps:30}}"

# C4 - both cameras (current config)
--robot.cameras="{wrist:{type:opencv,index_or_path:6,width:640,height:480,fps:30}, scene:{type:opencv,index_or_path:0,width:640,height:480,fps:30}}"

# C5 - wrist via RealSense native path (need serial number)
# First find serial: python -c "import pyrealsense2 as rs; ctx=rs.context(); [print(d.get_info(rs.camera_info.serial_number)) for d in ctx.devices]"
--robot.cameras="{wrist:{type:intelrealsense,serial_number_or_name:YOUR_SERIAL,width:640,height:480,fps:30}}"

# C6 - wrist(RealSense native) + scene(OpenCV)
--robot.cameras="{wrist:{type:intelrealsense,serial_number_or_name:YOUR_SERIAL,width:640,height:480,fps:30}, scene:{type:opencv,index_or_path:0,width:640,height:480,fps:30}}"
```

**Key comparisons**:
- C1 Hz = "pure robot SDK ceiling" — if this is still ≤5 Hz, robot SDK is the bottleneck
- C1 vs C4: total camera cost
- C3 vs C5: OpenCV vs native RealSense difference

---

### Group E — Isolate Dataset Write / Image Saver

**Base: A5 + B3 + best camera config from C**

```bash
# E1 - video=True, 1 thread per camera (minimal writer threads)
--dataset.video=True --dataset.num_image_writer_threads_per_camera=1

# E2 - video=True, 2 threads per camera
--dataset.num_image_writer_threads_per_camera=2

# E3 - video=True, 4 threads per camera
--dataset.num_image_writer_threads_per_camera=4

# E4 - video=False (save raw PNG, no encoding)
--dataset.video=False

# E5 - no cameras, record state/action only (pure write baseline)
# (same as C1 but now comparing write costs)
```

**Note**: RoboCOIN source comments explicitly warn that too many writer threads can **block the main thread** and reduce teleop FPS. More threads ≠ faster. Test empirically.

---

### Group F — rm_movej vs rm_movej_canfd (MOST IMPACTFUL)

**This is the most important code change.**

**Code change needed in `bi_realman.py`** — modify `_set_joint_state()`:

```python
# CURRENT CODE (find this in bi_realman.py):
def _set_joint_state(self, joint_state: np.ndarray):
    # ... unit conversion to joints_deg ...
    self.arm_left.rm_movej(joints_deg_left, self.config.velocity, 0, 0, int(self.config.block))
    self.arm_right.rm_movej(joints_deg_right, self.config.velocity, 0, 0, int(self.config.block))
    if not self.config.block:
        time.sleep(self.config.wait_second)
    # gripper calls...

# REPLACEMENT CODE (for F3/F4 experiments):
def _set_joint_state(self, joint_state: np.ndarray):
    # ... same unit conversion to joints_deg ...
    # NOTE: rm_movej_canfd takes degrees same as rm_movej, no unit change needed
    self.arm_left.rm_movej_canfd(joints_deg_left, follow=True, trajectory_mode=1, radio=50)
    self.arm_right.rm_movej_canfd(joints_deg_right, follow=True, trajectory_mode=1, radio=50)
    # NO sleep needed - canfd is fire-and-forget transparent passthrough
    # gripper calls remain the same (gripper uses separate channel)
```

**rm_movej_canfd signature** (from Realman Python SDK):
```python
rm_movej_canfd(
    joint: list[float],   # joint angles in DEGREES (same unit as rm_movej)
    follow: bool,          # True = high-follow mode (for teleop), False = normal
    expand: float = 0,     # extended joint (0 if not used)
    trajectory_mode: int = 0,  # 0=position, 1=smooth trajectory
    radio: int = 0         # smoothing factor 0-100 (50 is good default)
) -> int
```

**Experiments**:

```bash
# F1 - baseline: rm_movej, wait=0.1, vis=False, dual cameras
--robot.wait_second=0.1  # use rm_movej (don't change code yet)

# F2 - rm_movej, wait=0.0, vis=False, dual cameras  
--robot.wait_second=0.0  # still rm_movej

# F3 - rm_movej_canfd, wait=0.0, vis=False, dual cameras
# (change bi_realman.py to use rm_movej_canfd as above)

# F4 - rm_movej_canfd, wait=0.0, vis=False, NO cameras
# theoretical ceiling of canfd path
```

**Decision point**: F3 Hz is your practical ceiling with cameras. F4 is your ceiling without.

---

### Group G — Combined Optimal Config

Run after A–F to confirm final achievable Hz.

```bash
# G1 - All best settings combined
lerobot-record \
  --robot.type=bi_realman \
  --robot.ip_left=169.254.128.18 --robot.port_left=8080 \
  --robot.ip_right=169.254.128.19 --robot.port_right=8080 \
  --robot.block=False \
  --robot.wait_second=0.0 \
  --robot.visualize=False \
  --robot.draw_2d=False \
  --robot.draw_3d=False \
  --robot.init_type=none \
  --robot.cameras="{wrist:{type:intelrealsense,serial_number_or_name:YOUR_SERIAL,width:640,height:480,fps:30}, scene:{type:opencv,index_or_path:0,width:640,height:480,fps:30,fourcc:MJPG}}" \
  --teleop.type=bi_realman_leader \
  --teleop.ip_left=169.254.128.18 --teleop.port_left=8080 \
  --teleop.ip_right=169.254.128.19 --teleop.port_right=8080 \
  --teleop.init_type=none \
  --dataset.repo_id=YOUR_USER/realman_collect \
  --dataset.fps=15 \
  --dataset.episode_time_s=60 \
  --dataset.reset_time_s=10 \
  --dataset.num_episodes=5 \
  --dataset.push_to_hub=False \
  --dataset.num_image_writer_threads_per_camera=2  # tune based on E results
# (bi_realman.py already patched to use rm_movej_canfd from F experiments)
```

---

## 4. Results Tracking Table

Fill this in as you run experiments:

| Exp | Key Variable | Video len (s) | WCT (s) | Actual Hz | Notes |
|-----|-------------|--------------|---------|-----------|-------|
| Baseline | all defaults | | 20 | ~3-4 | starting point |
| D | SDK call timings | N/A | N/A | N/A | print ms per call |
| A1 | wait=0.1 | | 20 | | |
| A3 | wait=0.01 | | 20 | | |
| A5 | wait=0.0 | | 20 | | ← key decision point |
| B3 | vis=False | | 20 | | |
| C1 | no cameras | | 20 | | ← key decision point |
| C3 | wrist only (CV) | | 20 | | |
| C5 | wrist only (RS) | | 20 | | |
| C4 | dual cam (CV) | | 20 | | |
| C6 | dual cam (RS+CV) | | 20 | | |
| E1 | video=T, 1thread | | 20 | | |
| E3 | video=T, 4thread | | 20 | | |
| E4 | video=False | | 20 | | |
| F2 | movej, wait=0 | | 20 | | |
| F3 | canfd, wait=0 | | 20 | | ← most important fix |
| F4 | canfd, no cam | | 20 | | ← theoretical ceiling |
| G1 | all optimal | | 20 | | ← final target |

---

## 5. Code Changes Summary

### Change 1: configuration_bi_realman.py (safe defaults)
```python
# Change these defaults:
block: bool = False
wait_second: float = 0.0    # was 0.1
velocity: int = 50           # was 30
visualize: bool = False      # was True
draw_2d: bool = False        # was True
draw_3d: bool = False        # was True
```

### Change 2: configuration_bi_realman_leader.py (same defaults)
```python
wait_second: float = 0.0
velocity: int = 50
```

### Change 3: bi_realman.py — switch to rm_movej_canfd (for F3+ experiments)
```python
# In _set_joint_state(), replace:
self.arm_left.rm_movej(joints_deg_left, self.config.velocity, 0, 0, int(self.config.block))
if not self.config.block:
    time.sleep(self.config.wait_second)

# With:
self.arm_left.rm_movej_canfd(joints_deg_left, follow=True, trajectory_mode=1, radio=50)
# No sleep
```

### Change 4: Camera config — get RealSense serial number
```bash
python3 -c "
import pyrealsense2 as rs
ctx = rs.context()
for d in ctx.devices:
    print('Serial:', d.get_info(rs.camera_info.serial_number))
    print('Name:  ', d.get_info(rs.camera_info.name))
"
```
Then use `type: intelrealsense, serial_number_or_name: <serial>` in camera config.

### Change 5: record.py — add per-loop timing (temporary, for diagnostics)
```python
# Find the main while loop in record.py, add at the very top inside the loop:
import time as _time
_loop_start = _time.perf_counter()

# Add at the very bottom inside the loop (before busy_wait):
_loop_elapsed = (_time.perf_counter() - _loop_start) * 1000
print(f"[TIMING] total={_loop_elapsed:.1f}ms  est_hz={1000/_loop_elapsed:.1f}")
```

---

## 6. Architecture Fix (if experiments show SDK is the ceiling)

If F4 (canfd + no cameras) is still ≤8 Hz, the problem is TCP state polling. Implement UDP push:

```python
# In bi_realman.py connect method, after connecting arms:
from Robotic_Arm.rm_robot_interface import rm_realtime_push_config_t

# Configure arm to push state to this PC at 200Hz
config_left = rm_realtime_push_config_t(
    cycle=5,          # every 5ms = 200Hz push frequency
    enable=True,
    port=8099,        # left arm pushes to port 8099
    force_coordinate=0,
    ip='YOUR_PC_IP'   # e.g. 192.168.1.100
)
self.arm_left.rm_set_realtime_push(config_left)

config_right = rm_realtime_push_config_t(
    cycle=5,
    enable=True,
    port=8100,        # right arm pushes to port 8100 (different port!)
    force_coordinate=0,
    ip='YOUR_PC_IP'
)
self.arm_right.rm_set_realtime_push(config_right)
```

Then add a UDP receiver thread in `__init__`:
```python
import socket, struct, threading

class UDPJointReceiver:
    """Background thread that receives UDP state pushes from Realman arm."""
    def __init__(self, port: int, n_joints: int = 7):
        self._joints = np.zeros(n_joints + 1)  # 7 joints + gripper
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', port))
        self._sock.settimeout(0.5)
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
                # Parse UDP packet — check Realman UDP struct docs for exact byte layout
                # The realtimeArmJointState struct contains joint_status with joint angles
                # Adjust offset based on actual packet format from docs
                joints_deg = struct.unpack_from('7f', data, offset=4)  # verify offset!
                with self._lock:
                    self._joints[:7] = np.array(joints_deg)
            except socket.timeout:
                continue

    def get_joints(self) -> np.ndarray:
        with self._lock:
            return self._joints.copy()

    def stop(self):
        self._running = False
        self._thread.join()
        self._sock.close()
```

Replace `_get_joint_state()` to use the UDP receiver instead of TCP:
```python
def _get_joint_state(self) -> np.ndarray:
    # OLD (TCP polling, ~10-30ms):
    # result = self.arm.rm_get_current_arm_state()
    # return np.array(result[1]['joint'] + [gripper])

    # NEW (UDP shared memory, ~0.01ms):
    return self._udp_receiver.get_joints()
```

---

## 7. Three-Thread Architecture (if all above still insufficient)

If after all fixes you still can't hit 15Hz, implement full decoupled architecture:

```
Thread A (100Hz): control only
    leader.get_joints() → follower.rm_movej_canfd() → shared_state.update()

Thread B (30Hz): cameras only
    realsense.read() + webcam.read() → shared_state.update_cameras()

Thread C (15Hz): recording only
    shared_state.snapshot() → raw_buffer.append()

Main thread: orchestration + offline conversion to LeRobot format
```

This completely decouples control frequency from recording frequency.
Control runs at 100Hz for smooth following; recording logs at 15Hz for training data.

---

## 8. pi0.5 Training Config Based on Final Hz

| Achieved Hz | action_horizon | Notes |
|------------|----------------|-------|
| 3–4 Hz | 2–3 | minimal; proof of concept only |
| 8–10 Hz | 4–5 | workable for simple pick-and-place |
| 15 Hz | 8–10 | matches DROID protocol, ideal |
| 20–30 Hz | 10–16 | can use default OpenPI config |

---

## 9. Safe Recording Command (for actual data collection after fixes)

```bash
lerobot-record \
  --robot.type=bi_realman \
  --robot.ip_left=169.254.128.18 --robot.port_left=8080 \
  --robot.ip_right=169.254.128.19 --robot.port_right=8080 \
  --robot.block=False \
  --robot.wait_second=0.0 \
  --robot.velocity=50 \
  --robot.visualize=False \
  --robot.draw_2d=False \
  --robot.draw_3d=False \
  --robot.init_type=none \
  --robot.cameras="{
    wrist:{type:intelrealsense,serial_number_or_name:YOUR_SERIAL,width:640,height:480,fps:30},
    scene:{type:opencv,index_or_path:0,width:640,height:480,fps:30,fourcc:MJPG}
  }" \
  --teleop.type=bi_realman_leader \
  --teleop.ip_left=169.254.128.18 --teleop.port_left=8080 \
  --teleop.ip_right=169.254.128.19 --teleop.port_right=8080 \
  --teleop.init_type=none \
  --dataset.repo_id=YOUR_HF_USER/realman_pickplace \
  --dataset.fps=15 \
  --dataset.episode_time_s=90 \
  --dataset.reset_time_s=15 \
  --dataset.num_episodes=80 \
  --dataset.single_task="Pick up all silver connectors and place them into the white container" \
  --dataset.push_to_hub=False
```

---

## 10. Key Reminders

- Always set `--robot.init_type=none` AND `--teleop.init_type=none` — otherwise follower moves to hardcoded home pose on startup and may hit the camera tripod
- `dataset.episode_time_s` default is 60s — set explicitly
- Shell line continuations: **no spaces after `\`**
- `rm_movej_canfd` requires `follow=True` for teleop mode; `follow=False` is for trajectory replay
- Scene camera view is already finalized — do not change it
- Wrist RealSense serial number: run the pyrealsense2 detection script in Change 4 above to get it
- Target dataset size for meaningful pi0.5 fine-tune: **≥80 episodes** per task
