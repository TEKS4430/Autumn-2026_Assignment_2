# Spring-2026_Task_2

## Part A

### Overview

This assignment explores **sensor drift, noise, and fusion** using a TurtleBot3 Burger robot
simulated in Webots. The robot is equipped with a LiDAR scanner, an IMU (gyroscope +
accelerometer), wheel encoders (odometry), and an Astra RGB-D camera. The assignment is
divided into three tasks that build on each other.

Webots physics is ideal — motors execute commands perfectly. To make the simulation
realistic, two types of sensor error are deliberately introduced by the provided code:

- **Odometry drift** — the robot driver models a small wheel size mismatch (right wheel
  2.4% larger than left). The reported heading drifts from the true heading over time,
  accumulating approximately 6° of error after 3 circles.
- **IMU drift** — the noise injector adds a constant gyro bias of `0.015 rad/s` on the
  Z axis. This integrates to approximately 40° of heading error after 3 circles.

---

### Environment Setup

##### Please follow the [first assingment](https://github.com/TEKS4430/Spring-2026_Task_1) for .env setup on different OS**
The simulation runs in two parts:

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


---

### Task 2 — Sensor Filtering
 
**File:** `sensing_assignment/task2_filter.py`
 
**Goal:** Apply simple filters to clean up the noisy sensor data.
Publish cleaned versions that Task 3 will use.
 
**What to implement:**
 
1. **TODO 2a** — Running average filter on IMU gyro Z from `/imu_noisy`.
   Update a single running average value each time a new reading arrives:
   ```python
   self.gyro_z_avg = 0.9 * self.gyro_z_avg + 0.1 * msg.angular_velocity.z
   ```
   Copy the IMU message, replace `angular_velocity.z` with `self.gyro_z_avg`,
   and publish on `/imu_filtered`.
 
2. **TODO 2b** — Invalid ray removal on `/scan_noisy`.
   Replace any ray that is `inf` or below `range_min` with `0.0`,
   then publish the cleaned scan on `/scan_filtered`.
   ```python
   ranges = list(msg.ranges)
   for i in range(len(ranges)):
       if math.isinf(ranges[i]) or ranges[i] < msg.range_min:
           ranges[i] = 0.0
   ```

### Task 3 — Sensor Fusion

**File:** `sensing_assignment/task3_fusion.py`

**Goal:** Combine odometry and IMU to get a better heading estimate than either
sensor alone, using a complementary filter.

**Why both sensors drift:**

Odometry is computed from wheel encoder readings. The robot's wheels are modelled
with a small size mismatch (left and right wheels have slightly different radii),
which is realistic — real robots are never perfectly calibrated. Equal motor commands
produce slightly unequal distances, so the reported heading slowly drifts from the
true heading over time.

The IMU gyroscope has a constant bias injected by `noise_injector.py`
(`0.015 rad/s` on the Z axis). Even when the robot is stationary, the gyro reports
a small non-zero rotation rate. When this is integrated over time to get heading,
the error grows linearly — approximately `13.5°` per circle at the commanded speed.

**Why fusion helps:**

Neither sensor alone is reliable over many circles. But their errors have different
characters — odometry accumulates slowly and consistently, IMU accumulates faster
but could in principle be corrected. A complementary filter blends both:

```
yaw_fused = alpha * yaw_odom + (1 - alpha) * yaw_imu_integrated
```

**Tuning alpha:**
- `alpha = 1.0` — use only odometry (ignores IMU entirely)
- `alpha = 0.0` — use only IMU integration (accumulates bias rapidly)
- `alpha = 0.98` — trust odometry heavily, small IMU correction
- `alpha = 0.70` — stronger IMU contribution, may help or hurt depending on drift

There is no single correct value — students should experiment and report what they find.

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

**How to evaluate:**
The robot drives in circles. After each circle it should return to heading `0°`
(equivalently `360°`). The console prints all three headings every 3 seconds.
Expected drift after 3 circles (approx. 47 seconds of driving):

```
Odom heading:  ~354°   → ~6° error   (wheel radius mismatch)
IMU heading:   ~400°   → ~40° error  (gyro bias 0.015 rad/s × 47s integrated)
Fused (α=0.98) ~354°   → ~6° error   (mostly odometry — fusion barely changes it)
Fused (α=0.70) ~367°   → ~7° error   (too much IMU weight makes things worse here)
```

This shows that with a large gyro bias and moderate odometry drift, high alpha
(trusting odometry) is the right strategy. The interesting question for students
is: when would low alpha (trusting IMU more) be better?

**Questions to answer in your report:**

- Q1. What is the heading error from odometry alone after 3 circles?
- Q2. What is the heading error from IMU integration alone after 3 circles?
  Why is it so much larger than odometry?
- Q3. What value of `alpha` gave the best fused result? Explain why.
- Q4. The complementary filter does not remove the gyro bias — it only reduces
  its effect. What would be needed to fully correct for a constant gyro bias?
- Q5. In a real robot without injected noise, which sensor would you trust more
  for short-term heading estimation — odometry or gyro? Why?

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

## Part B: Use of AI tools and self-reflection
In part B, you will reflect on the new skills you have learned and analyze your own work and learning process. You will consider what technical skills you learned while completing part A, what tools you used, why you chose to use them, and what benefits or challenges were associated with their use.

If you did not use AI tools, you should focus on self-reflection regarding your own learning process and discuss your motivations for not using AI as a tool.

The use of AI tools is also allowed in completing part B. However, you should still take responsibility for your own learning and approach the reporting seriously. Ultimately, you are studying for your own benefit.

The use of AI must be reported transparently and appropriately (this is also required by university policies). When completing Sections A and B (note: AI use is generally prohibited in part C), you must keep detailed records of:

- which AI tools you used (e.g., Claude, ChatGPT, Copilot, Gemini, Codex, VSCode extensions, etc.),

- the specific language model version (e.g., Sonnet 4.6, GPT-5.4),

- and the settings or modes used (e.g., thinking mode, instant mode, or agent mode).

You may also use any other models or technologies, such as locally running language models in your own environment (for example, models run through environments like Ollama) or other AI agent systems.

The primary motivation of this section is to encourage continuous reflection on your learning, both in terms of technical skills and the technologies used to implement solutions.


## Returning instructions

The assignment must be submitted in video format. Create a video that reports Parts A and B, demonstrating that you have successfully set up the environment, explaining how you learned to use your codebase, and presenting your self-reflection as well as describing how you used AI in the assignment.

In the video, use software such as Microsoft Teams to record your screen while presenting and recording your voice. In the recording, you should demonstrate that your environment is running correctly and show the most essential parts of your codebase. In addition, you should use PowerPoint, Google Slides, or a similar tool to document Part B and present it in the video.

This process will not only teach you how to present your work to others, but it will also help facilitate peer learning and support among students. Although in this first assignment there is only a small amount of technical implementation to report, you will learn the submission procedure that will be used for the rest of the assignments, which will be much more implementation-oriented.

We recommend using Microsoft Teams, as it allows you to record both your screen and voice. The recordings are automatically uploaded to SharePoint, which makes it easy to share your recording later for the peer review assignment. Alternatively, you may use other software such as QuickTime Player or OBS to record your video and then upload the recording to SharePoint.

**Important:** You must ensure that your recording is accessible to others who have the link. So, via Sharepoint user interface in your browser (see below) define the shared settings so that anyone who has the link can access the file for maximum number of days. Finally, you should ensure for example in privacy mode or another browser that the link is truly accessible without login.

![create_sharelink](https://github.com/TEKS4430/Spring-2026_Task_1/blob/main/screenshots/accessrights.png)

<p align="center">
<img src="https://github.com/TEKS4430/Spring-2026_Task_1/blob/main/screenshots/link_settings.png" width=50% height=50%>    
</p>
