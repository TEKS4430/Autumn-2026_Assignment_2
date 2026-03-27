# Spring-2026_Task_2

## Part A

### Overview

This assignment explores **sensor drift, noise, and fusion** using a TurtleBot3 Burger robot
simulated in Webots. The robot is equipped with a LiDAR scanner, an IMU (gyroscope +
accelerometer), wheel encoders (odometry), and an Astra RGB-D camera. The assignment is
divided into three tasks that build on each other.

---

### Environment Setup

The simulation runs in two parts:

- **Webots** (runs on your host machine) — provides the 3D simulation
- **ROS2 Jazzy** (runs inside a Docker container) — processes sensor data

Start Webots and open the world file, then inside the container:

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select sensing_assignment
source install/setup.bash
ros2 launch sensing_assignment assignment.launch.py
```

To start the robot driving in a circle:

```bash
ros2 service call /start_driving std_srvs/srv/Trigger {}
```

To stop the robot:

```bash
ros2 service call /stop_driving std_srvs/srv/Trigger {}
```

---

### What Gets Launched

| Node | Type | Purpose |
|---|---|---|
| `WebotsController` | Provided | Connects Webots robot to ROS2. Publishes `/scan`, `/imu`, `/odom`. Subscribes to `/cmd_vel`. |
| `motion_controller` | Provided | Drives robot in a circle when triggered via service |
| `noise_injector` | Provided | Adds realistic gyro drift and LiDAR outliers on top of raw sensor data |
| `task1_observer` | **Student** | Subscribe to sensors and observe raw data |
| `task2_filter` | **Student** | Filter noisy sensor data |
| `task3_fusion` | **Student** | Fuse odometry and IMU for better heading estimate |

---

### Topic Map

```
Webots simulation
    ├── /scan          ← raw LiDAR (10 Hz)
    ├── /imu           ← raw IMU: gyro + accelerometer (50 Hz)
    └── /odom          ← wheel odometry: x, y, heading (50 Hz)

noise_injector (provided)
    ├── /scan  → /scan_noisy    adds outlier readings and dropouts
    └── /imu   → /imu_noisy    adds constant gyro bias + gaussian noise

Task 1 — observe
    subscribes to: /scan_noisy, /imu_noisy, /odom

Task 2 — filter
    subscribes to: /scan_noisy, /imu_noisy
    publishes:     /scan_filtered, /imu_filtered

Task 3 — fuse
    subscribes to: /odom, /imu_filtered
    publishes:     /pose_fused
```

---

### Task 1 — Sensor Observation

**File:** `sensing_assignment/task1_observer.py`

**Goal:** Subscribe to sensor topics and observe what the data looks like in practice.
The robot drives in a circle. After one complete circle it should return to its
starting position — but it does not. This is sensor drift.

**What to implement** (marked with `# TODO` in the file):

1. **TODO 1a** — Extract `x`, `y`, and `yaw` from the `/odom` message and log them.
   - Position is in `msg.pose.pose.position`
   - Orientation is a quaternion: convert to yaw using
     `yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))`

2. **TODO 1b** — Collect `angular_velocity.z` samples from `/imu_noisy`.
   Store them in `self.imu_gyro_z_samples`.

3. **TODO 1c** — Count total and outlier LiDAR rays from `/scan_noisy`.
   An outlier is: `inf`, `NaN`, below `range_min`, or above `range_max`.

4. **TODO 1d** — Every 5 seconds, print mean and standard deviation of
   gyro Z samples, and the outlier percentage in the LiDAR scan.
   Clear the sample buffer after printing.

**Questions to answer in your report:**

- Q1. What is the odometry position after 3 full circles? Should it be `(0, 0)`?
- Q2. What is the mean and standard deviation of `angular_velocity.z` when the
  robot is stationary? What does a non-zero mean tell you?
- Q3. What percentage of LiDAR rays appear to be outliers? How did you identify them?

**Hint — computing standard deviation without numpy:**
```python
mean = sum(samples) / len(samples)
std  = math.sqrt(sum((x - mean)**2 for x in samples) / len(samples))
```

---

### Task 2 — Sensor Filtering

**File:** `sensing_assignment/task2_filter.py`

**Goal:** Implement filters to clean up noisy sensor data and publish cleaned versions.

**What to implement:**

1. **TODO 2a** — Moving average filter on `angular_velocity.z` from `/imu_noisy`.
   - Keep a sliding window of the last N readings (`self.gyro_z_window`, size 10)
   - Replace each new reading with the window average
   - Publish the result on `/imu_filtered`
   - **Question:** Does the moving average remove the constant gyro bias? Why not?

2. **TODO 2b** — Outlier rejection on `/scan_noisy`.
   - For each range value: if it is `inf`, `NaN`, below `range_min`, or above
     `range_max`, replace it with `0.0`
   - Publish the cleaned scan on `/scan_filtered`
   - **Bonus:** Implement a 3-point median filter — replace each ray with the
     median of itself and its two neighbours

**Hint — copying a ROS2 message:**
You cannot modify a received message directly. Create a new one and copy fields:
```python
from sensor_msgs.msg import Imu
filtered = Imu()
filtered.header = msg.header
filtered.angular_velocity = msg.angular_velocity  # then modify z
```

**Hint — checking for invalid float values:**
```python
import math
if math.isinf(value) or math.isnan(value):
    # invalid reading
```

---

### Task 3 — Sensor Fusion

**File:** `sensing_assignment/task3_fusion.py`

**Goal:** Combine odometry and IMU to get a better heading estimate than either
sensor alone, using a complementary filter.

**The problem:**
- Odometry heading drifts slowly — it accumulates error from wheel slip
- IMU gyro heading drifts faster — it has a constant bias that integrates over time
- A complementary filter blends both to reduce the total error

**The complementary filter:**
```
yaw_fused = alpha * yaw_odom + (1 - alpha) * yaw_imu_integrated
```

where `alpha = 0.98` means "trust odometry 98%, correct slowly with IMU."

**What to implement:**

1. **TODO 3a** — Extract `x`, `y`, and `yaw_odom` from `/odom`.
   Convert quaternion to yaw and store in `self.yaw_odom`.
   Call `self.publish_fused_pose()` at the end.

2. **TODO 3b** — Integrate gyro to estimate heading from `/imu_filtered`.
   Compute `dt` from message timestamps:
   ```python
   dt = (current.sec - last.sec) + (current.nanosec - last.nanosec) * 1e-9
   self.yaw_imu += msg.angular_velocity.z * dt
   ```

3. **TODO 3c** — Apply the complementary filter and publish `/pose_fused`:
   ```python
   self.yaw_fused = self.alpha * self.yaw_odom + (1 - self.alpha) * self.yaw_imu
   ```
   Build a `PoseStamped` message with `x = self.x`, `y = self.y`, and convert
   `yaw_fused` to a quaternion:
   ```python
   msg.pose.orientation.z = math.sin(self.yaw_fused / 2)
   msg.pose.orientation.w = math.cos(self.yaw_fused / 2)
   ```

**How to evaluate without ground truth:**
The robot drives in a circle and should return to heading `0°` (= 360°) after
one full lap. The console prints heading from all three sources every 3 seconds.
After completing circles, compare:

```
Odom heading:  358.4°   → 1.6° error
IMU heading:   364.2°   → 4.2° error (bias accumulated)
Fused heading: 359.6°   → 0.4° error  ← best
```

**Questions to answer in your report:**

- Q1. What happens when `alpha = 1.0` (only odometry)?
- Q2. What happens when `alpha = 0.0` (only IMU integration)?
- Q3. What value of `alpha` gave the smallest heading error after 3 circles?
- Q4. Does the complementary filter remove the gyro bias? What would be needed
  to fully remove a constant bias?

---

### Verifying Your Work

Check that your topics are publishing:

```bash
ros2 topic list
ros2 topic hz /odom          # should show ~50 Hz
ros2 topic hz /imu_filtered  # should show ~13 Hz
ros2 topic hz /pose_fused    # should show ~50 Hz
```

Echo a topic to see values changing while the robot drives:

```bash
ros2 topic echo /odom --field pose.pose.position
```

---

### File Structure

```
sensing_assignment/
├── sensing_assignment/
│   ├── robot_driver.py       # Webots plugin: motors + odometry (DO NOT MODIFY)
│   ├── motion_controller.py  # Drives robot in a circle (DO NOT MODIFY)
│   ├── noise_injector.py     # Adds sensor noise (DO NOT MODIFY)
│   ├── task1_observer.py     # ← YOUR WORK: Task 1
│   ├── task2_filter.py       # ← YOUR WORK: Task 2
│   └── task3_fusion.py       # ← YOUR WORK: Task 3
├── launch/
│   └── assignment.launch.py
└── resource/
    ├── TurtleBot3Burger.urdf
    └── ros2control.yml
```