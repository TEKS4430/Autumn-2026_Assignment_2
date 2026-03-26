#!/usr/bin/env python3
"""
task3_fusion.py  — STUDENT TASK 3

Goal: Fuse odometry and IMU to get a better heading estimate
      than either sensor alone.

The problem:
    Odometry alone:     heading drifts because wheel slip and
                        small encoder errors accumulate over time.
    IMU (gyro) alone:   high-rate heading updates, but the gyro
                        has a constant bias → heading drifts even faster.

The insight:
    Odometry drift is slow and proportional to distance travelled.
    Gyro drift is fast and proportional to time.
    A complementary filter combines the best of both:
        yaw_fused = alpha * yaw_odom + (1 - alpha) * yaw_imu

How to evaluate without ground truth:
    The robot is commanded to drive in a circle and return to start.
    After N complete circles, the robot should be at (0,0) facing
    its original direction (yaw = 0).
    Measure:
        odom heading error    = |yaw_odom  - 0| after N circles
        fused heading error   = |yaw_fused - 0| after N circles
    The fused error should be smaller.

Input topics:
    /odom          (nav_msgs/Odometry)
    /imu_filtered  (sensor_msgs/Imu)   — use filtered IMU from Task 2

Output topics:
    /pose_fused    (geometry_msgs/PoseStamped)  — your fused estimate

Questions to answer in your report:
    Q1. After 3 circles, what is the heading error from odometry alone?
    Q2. What is the heading error from IMU integration alone?
    Q3. What is the heading error from your fused estimate?
    Q4. What value of alpha worked best? Why?
    Q5. Does the complementary filter remove the gyro bias?
        What would be needed to fully remove it?
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import math


class PoseFusion(Node):

    def __init__(self):
        super().__init__("pose_fusion")

        # ── Complementary filter weight ───────────────────────
        # alpha close to 1.0 = trust odometry heading more
        # alpha close to 0.0 = trust IMU integration more
        # TODO: experiment with values between 0.0 and 1.0
        self.alpha = 0.98

        # ── State ─────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.yaw_odom = 0.0  # heading from odometry
        self.yaw_imu = 0.0  # heading integrated from gyro
        self.yaw_fused = 0.0  # complementary filter output

        self.last_imu_stamp = None
        self.circle_count = 0
        self.last_yaw = 0.0  # for counting circles

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Imu, "/imu_filtered", self.imu_callback, 10)

        # ── Publisher ─────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, "/pose_fused", 10)

        # Print comparison every 3 seconds
        self.create_timer(3.0, self.print_comparison)

        self.get_logger().info(
            f"PoseFusion started  alpha={self.alpha}\n"
            "  Publishing /pose_fused\n"
            "  Observe heading drift after each circle in the console."
        )

    # ─────────────────────────────────────────────────────────
    # TODO 3a: Odometry callback
    #
    # Extract from msg:
    #   x, y   → msg.pose.pose.position.x / .y
    #   yaw    → convert quaternion to yaw:
    #     q = msg.pose.pose.orientation
    #     yaw = atan2(2*(q.w*q.z + q.x*q.y),
    #                 1 - 2*(q.y*q.y + q.z*q.z))
    #
    # Store in self.x, self.y, self.yaw_odom
    # Then call self.publish_fused_pose()
    # ─────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        # TODO: extract position and yaw from odometry
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 3b: IMU callback — integrate gyro to estimate heading
    #
    # 1. Compute dt using message timestamps:
    #      current = msg.header.stamp
    #      if self.last_imu_stamp is None: save and return
    #      dt = (current.sec - last.sec) +
    #           (current.nanosec - last.nanosec) * 1e-9
    # 2. Integrate:
    #      self.yaw_imu += msg.angular_velocity.z * dt
    # 3. Update self.last_imu_stamp = msg.header.stamp
    # ─────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        # TODO: integrate gyro angular_velocity.z over time
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 3c: Complementary filter and publish
    #
    # 1. Fuse headings:
    #      self.yaw_fused = (self.alpha * self.yaw_odom +
    #                        (1 - self.alpha) * self.yaw_imu)
    #
    # 2. Build PoseStamped:
    #      position: x=self.x, y=self.y, z=0.0
    #      orientation (yaw only):
    #        qz = sin(yaw_fused / 2)
    #        qw = cos(yaw_fused / 2)
    #        qx = qy = 0.0
    #
    # 3. Publish on /pose_fused
    # ─────────────────────────────────────────────────────────
    def publish_fused_pose(self):
        # TODO: compute fused heading and publish
        pass

    def print_comparison(self):
        """
        Print heading estimates to console for comparison.
        After each complete circle (360°) the robot should be
        back to yaw ≈ 0. Any deviation is the drift error.
        """
        self.get_logger().info(
            "\n--- Heading comparison ---\n"
            f"  Odom heading:  {math.degrees(self.yaw_odom):+.2f}°\n"
            f"  IMU heading:   {math.degrees(self.yaw_imu):+.2f}°\n"
            f"  Fused heading: {math.degrees(self.yaw_fused):+.2f}°\n"
            f"  Position (odom): x={self.x:.3f}m  y={self.y:.3f}m\n"
            "  After 1 full circle all headings should read ~360°\n"
            "  Deviation from 360° = drift error"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
