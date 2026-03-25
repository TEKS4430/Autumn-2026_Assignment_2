#!/usr/bin/env python3
"""
task2_filter.py  — STUDENT TASK 2

Goal: Clean up the noisy sensor data by implementing filters.
      Publish cleaned versions of each sensor topic.

Input topics:
    /imu_noisy   (sensor_msgs/Imu)       — raw IMU with noise + drift
    /scan_noisy  (sensor_msgs/LaserScan) — LiDAR with outliers

Output topics (you publish these):
    /imu_filtered   (sensor_msgs/Imu)       — cleaned IMU
    /scan_filtered  (sensor_msgs/LaserScan) — cleaned LiDAR

Filters to implement:

    IMU — Moving average filter on angular_velocity.z
        Keep a window of the last N gyro readings.
        Replace each reading with the average of the window.
        Effect: smooths out random noise.
        Question: does it remove the drift (bias)? Why or why not?

    LiDAR — Outlier rejection
        If a reading is inf, NaN, < range_min, or > range_max → replace with 0.0
        Optional: also apply a median filter across neighbouring rays.

Deliverable:
    This node must publish /imu_filtered and /scan_filtered.
    Your task3_fusion.py will use /imu_filtered as input.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import math
from collections import deque


class SensorFilter(Node):

    def __init__(self):
        super().__init__("sensor_filter")

        # ── Parameters ────────────────────────────────────────
        # Moving average window size for IMU
        self.window_size = 10  # TODO: experiment with different values

        # ── State ─────────────────────────────────────────────
        self.gyro_z_window = deque(maxlen=self.window_size)

        # ── Subscribers ───────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, "/imu_noisy", self.imu_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan_noisy", self.scan_callback, 10
        )

        # ── Publishers ────────────────────────────────────────
        self.imu_pub = self.create_publisher(Imu, "/imu_filtered", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan_filtered", 10)

        self.get_logger().info(
            "SensorFilter started.\n"
            "  /imu_noisy   → /imu_filtered\n"
            "  /scan_noisy  → /scan_filtered"
        )

    # ─────────────────────────────────────────────────────────
    # TODO 2a: IMU moving average filter
    #
    # 1. Add msg.angular_velocity.z to self.gyro_z_window
    # 2. Compute the mean of self.gyro_z_window
    # 3. Create a new Imu message (copy the original)
    # 4. Replace angular_velocity.z with the moving average
    # 5. Publish on /imu_filtered
    #
    # Question: compare the filtered vs raw gyro_z values.
    # Does the filter remove the constant bias? Why not?
    # How would you remove a constant bias?
    # ─────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        # TODO: implement moving average filter on gyro Z
        # Then publish the filtered message
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 2b: LiDAR outlier rejection
    #
    # 1. Copy the incoming LaserScan message
    # 2. For each range value:
    #    - If it is inf, NaN, < range_min, or > range_max
    #      → replace it with 0.0 (indicates invalid)
    # 3. Publish the cleaned scan on /scan_filtered
    #
    # Optional (bonus): implement a 3-point median filter
    #   For each ray i, replace ranges[i] with the median of
    #   ranges[i-1], ranges[i], ranges[i+1]
    # ─────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        # TODO: implement outlier rejection
        # Then publish the filtered message
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SensorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
