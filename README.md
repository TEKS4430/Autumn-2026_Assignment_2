# Autumn-2026_Assignment_2

## Part A: Studying and technical implementation — 4 points

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

##### Please follow the [first assingment](https://github.com/TEKS4430/Autumn-2026_Assignment_1) for .env setup on different OS**
The simulation runs in two parts:

- **Webots** (runs on your host machine) — provides the 3D simulation
- **ROS2 Jazzy** (runs inside a Docker container) — processes sensor data

Start Webots and open the world file.

###### Note: When opening the world file, Webots may show errors about missing PROTO files and the robot may appear white. This is a cosmetic issue caused by network connectivity to GitHub's asset server. The simulation functions correctly — the robot, LiDAR, and all sensors work normally. Closing the webots simulator and reopening it works for me.



Then inside the container:

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


###### Note: If the simulation seems too heavy for your machine, for example lags a lot or freezes everything, try removing the camera from the TurtleBot3Burger node on the left panel. You can find the camera under TurtleBot3Burger -> extensionSlot -> Astra. The camera is just for visual purpose, you won't need that for any assignment task. You can also remove objects from the scene to make it more simple to render.

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

---

## Part B: Learning process, use of AI tools, and self-reflection — 4 points

Part B is an equally important part of the assignment as the technical implementation in Part A. The purpose is to reflect on **what you learned, how you worked, and how AI or other tools and resources affected your learning and problem-solving process**.

You are **not assessed based on how much AI you use**. You may use AI extensively, selectively, or not at all. Instead, you are assessed based on the quality of your reflection, your critical evaluation of your working and learning process, and your demonstrated understanding and ownership of the final solution.

### If you did not use AI

Using AI is **not required and does not affect the maximum number of points available**.

If you chose not to use AI, explain why and focus instead on **how you learned and solved the assignment independently**. Describe the resources, tools, documentation, experimentation, debugging, discussions, or other approaches you used to understand the technical concepts and overcome problems.

The same assessment criteria apply: the important thing is to demonstrate and critically reflect on **how you worked, what you learned, and how you evaluated the correctness of your solution**.

In your presentation, address the following four areas:

### 1. What did you learn?

Describe the most important things you learned while completing Part A.

Focus especially on your **technical learning**. For example, in this assignment you might discuss what you learned about sensor data, noise and drift, filtering, sensor fusion, ROS2 topics, or implementing and debugging the provided code.

Do not simply describe what you did. Explain **what you understand now that you did not understand before the assignment**.

**If you used AI:** Explain how AI helped you understand new concepts, code, technologies, or problems and how your understanding developed through this interaction.

**If you did not use AI:** Explain how you developed this understanding. For example, did you study course materials or documentation, search for examples, experiment with the system, inspect code, discuss problems with others, or learn through trial and error?

### 2. How did you solve problems and use AI, tools, and other resources?

Describe **how you approached the assignment and solved the technical problems you encountered**.

**If you used AI**, explain the significant ways in which AI contributed to your work. For each important use, consider:

- what you were trying to accomplish or understand;
- which tool and model you used;
- why you decided to use AI for this particular purpose;
- how you used it; and
- how useful the result was.

You do not need to report every individual prompt. Instead, focus on the **most important ways in which AI influenced your work**.

You must nevertheless transparently identify the AI tools and models you used in Parts A and B, including the model/version and relevant mode or settings when this information is available.

For example:

> I used ChatGPT with GPT-X in thinking mode to understand why integrating a small constant gyroscope bias produces an increasing heading error. I then used it to help interpret the results produced by my implementation.

**If you did not use AI:** Describe the corresponding problem-solving process. What resources or tools did you use instead? How did you find the information you needed? How did you approach debugging or unfamiliar technical concepts? Explain why you chose these approaches and how useful they were.

### 3. Critically evaluate your tools and problem-solving process

Choose at least **one concrete example of a problem, challenge, or learning situation** from the assignment and analyze how you addressed it.

**If you used AI**, this could be a situation where AI:

- significantly helped you;
- produced an incomplete or incorrect answer;
- suggested an approach that needed modification; or
- provided information whose correctness you needed to verify.

Explain what AI suggested, whether the suggestion was useful, **how you determined whether you could trust it**, and how you tested, modified, improved, or rejected the suggestion.

**If you did not use AI:** Select a corresponding technical challenge and explain how you investigated and solved it. For example, describe how you used documentation, course materials, experimentation, debugging, testing, online resources, or discussions with others. Consider whether your initial approach worked, how you evaluated different alternatives, and how you verified that the final solution was correct.

The purpose is to demonstrate that you can **critically evaluate information, tools, and proposed solutions rather than simply accepting them**.

### 4. Reflect on your own learning and working process

Finally, reflect on your overall experience and how you approached learning during the assignment.

Consider questions such as:

- What was the most difficult part of the assignment?
- What did you understand better after completing it?
- Which problem-solving or learning approaches worked particularly well?
- Which approaches did not work well?
- How did you verify that you understood the technical concepts rather than merely producing a working solution?
- If you completed the assignment again, what would you do differently?

**If you used AI:** Also consider whether AI made you more productive or helped you learn, whether it introduced additional problems or confusion, and whether there were situations where solving or studying something without AI was more useful.

**If you did not use AI:** Reflect on how effective your chosen learning and problem-solving strategies were. Consider whether documentation, experimentation, debugging, course materials, discussions, or other resources supported your learning effectively and whether another approach could have made your work or learning more effective.

Your reflection should demonstrate that **you understand and take responsibility for the work you submitted**, regardless of whether or how much AI contributed to the process.

---


## Returning instructions

The assignment must be submitted in video format. Create a video that reports Parts A and B, demonstrating that you have successfully set up the environment, explaining how you learned to use your codebase, and presenting your self-reflection as well as describing how you used AI in the assignment.

In the video, use software such as Microsoft Teams to record your screen while presenting and recording your voice. In the recording, you should demonstrate that your environment is running correctly and show the most essential parts of your codebase. In addition, you should use PowerPoint, Google Slides, or a similar tool to document Part B and present it in the video.

This process will not only teach you how to present your work to others, but it will also help facilitate peer learning and support among students. Although in this first assignment there is only a small amount of technical implementation to report, you will learn the submission procedure that will be used for the rest of the assignments, which will be much more implementation-oriented.

We recommend using Microsoft Teams, as it allows you to record both your screen and voice. The recordings are automatically uploaded to SharePoint, which makes it easy to share your recording later for the peer review assignment. Alternatively, you may use other software such as QuickTime Player or OBS to record your video and then upload the recording to SharePoint.

**Important:** You must ensure that your recording is accessible to others who have the link. So, via Sharepoint user interface in your browser (see below) define the shared settings so that anyone who has the link can access the file for maximum number of days. Finally, you should ensure for example in privacy mode or another browser that the link is truly accessible without login.

![create_sharelink](https://github.com/TEKS4430/Autumn-2026_Assignment_1/blob/main/screenshots/accessrights.png)

<p align="center">
<img src="https://github.com/TEKS4430/Autumn-2026_Assignment_1/blob/main/screenshots/link_settings.png" width=50% height=50%>    
</p>
