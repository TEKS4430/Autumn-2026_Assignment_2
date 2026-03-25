#!/usr/bin/env python3
"""
task3_fusion.py  — STUDENT TASK 3

Goal: Fuse odometry and IMU to get a better pose estimate than
      either sensor alone.

The problem:
    /odom alone:      accurate position in short term, heading drifts
    /imu_filtered:    high-rate heading estimate, but has gyro bias → drifts

The idea (complementary filter):
    Use odometry for position (x, y).
    Use a complementary filter to improve heading (yaw):
        yaw_fused = alpha * yaw_from_odom + (1 - alpha) * yaw_from_imu_integration

    where:
        yaw_from_odom = heading computed from odometry twist
        yaw_from_imu  = heading integrated from gyro angular_velocity.z
        alpha = 0.98   (trust odom heading mostly, correct slowly with IMU)

Input topics:
    /odom          (nav_msgs/Odometry)  — wheel odometry
    /imu_filtered  (sensor_msgs/Imu)   — filtered IMU from Task 2

Output topics:
    /pose_estimate  (geometry_msgs/PoseStamped)  — your fused estimate

Evaluation:
    Compare /pose_estimate against /ground_truth_pose.
    The error should be smaller than using /odom alone.

Questions to answer in your report:
    Q1. What happens when alpha = 1.0 (only odometry)?
    Q2. What happens when alpha = 0.0 (only IMU integration)?
    Q3. What value of alpha gives the best result and why?
    Q4. Does the complementary filter remove the gyro bias?
        What would be needed to fully correct for it?
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Header
import math


class PoseFusion(Node):

    def __init__(self):
        super().__init__("pose_fusion")

        # ── Complementary filter parameter ────────────────────
        # TODO: experiment with different values (0.0 to 1.0)
        self.alpha = 0.98  # weight for odometry heading

        # ── State ─────────────────────────────────────────────
        # Position from odometry
        self.x = 0.0
        self.y = 0.0

        # Heading estimates
        self.yaw_odom = 0.0  # from odometry
        self.yaw_imu = 0.0  # integrated from gyro
        self.yaw_fused = 0.0  # complementary filter output

        # For IMU integration
        self.last_imu_time = None

        # ── Subscribers ───────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu_filtered", self.imu_callback, 10
        )

        # ── Publisher ─────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, "/pose_estimate", 10)

        # Print comparison every 3 seconds
        self.create_timer(3.0, self.print_comparison)

        self.get_logger().info(
            f"PoseFusion started. alpha={self.alpha}\n" "  Publishing /pose_estimate"
        )

    # ─────────────────────────────────────────────────────────
    # TODO 3a: Odometry callback
    #
    # 1. Extract x, y from msg.pose.pose.position
    # 2. Extract yaw from msg.pose.pose.orientation (quaternion → yaw)
    #    yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    # 3. Store in self.x, self.y, self.yaw_odom
    # 4. Call self.publish_fused_pose()
    # ─────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        # TODO: extract position and heading from odometry
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 3b: IMU callback — integrate angular velocity to get heading
    #
    # 1. Compute dt = time since last IMU message (in seconds)
    #    Use msg.header.stamp (ROS2 timestamp)
    #    dt = (current_time - last_time).nanoseconds / 1e9
    # 2. Integrate: self.yaw_imu += msg.angular_velocity.z * dt
    # 3. Update self.last_imu_time
    #
    # Note: this integration accumulates error over time (drift).
    # The complementary filter below corrects for this.
    # ─────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        # TODO: integrate gyro to update self.yaw_imu
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 3c: Complementary filter + publish
    #
    # 1. Compute fused yaw:
    #    self.yaw_fused = self.alpha * self.yaw_odom +
    #                     (1 - self.alpha) * self.yaw_imu
    #
    # 2. Build a PoseStamped message:
    #    - position: x = self.x, y = self.y, z = 0.0
    #    - orientation: convert yaw_fused to quaternion
    #      qz = sin(yaw/2), qw = cos(yaw/2), qx=qy=0
    #
    # 3. Publish on /pose_estimate
    # ─────────────────────────────────────────────────────────
    def publish_fused_pose(self):
        # TODO: compute fused yaw and publish PoseStamped
        pass

    def print_comparison(self):
        """Print current estimates to console for comparison."""
        self.get_logger().info(
            f"[Pose estimates]\n"
            f"  Odom heading:  {math.degrees(self.yaw_odom):+.1f}°\n"
            f"  IMU heading:   {math.degrees(self.yaw_imu):+.1f}°\n"
            f"  Fused heading: {math.degrees(self.yaw_fused):+.1f}°\n"
            f"  Position: x={self.x:.3f}  y={self.y:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
