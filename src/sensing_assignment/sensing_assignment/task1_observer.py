#!/usr/bin/env python3
"""
task1_observer.py  — STUDENT TASK 1

Goal: Subscribe to sensor topics and observe raw data.
      Print key values to the console. Identify sensor issues.

Topics to subscribe to:
    /odom        (nav_msgs/Odometry)    — wheel odometry
    /imu_noisy   (sensor_msgs/Imu)      — IMU with injected noise/drift
    /scan_noisy  (sensor_msgs/LaserScan) — LiDAR with injected outliers

What to observe:
    1. What is the robot's position according to odometry after one circle?
       Is it back at (0, 0)? Why or why not?
    2. What does the IMU angular_velocity.z show when the robot is stationary?
       Is it zero? What does a non-zero value mean?
    3. Look at the LiDAR scan ranges. Do any readings look wrong?
       What values indicate a failed/outlier reading?

Questions to answer in your report:
    Q1. What is the odometry position error after 3 full circles?
    Q2. What is the mean and standard deviation of angular_velocity.z
        when the robot is stationary? What does this tell you about the sensor?
    Q3. What percentage of LiDAR rays appear to be outliers?
        How did you identify them?
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import math


class SensorObserver(Node):

    def __init__(self):
        super().__init__("sensor_observer")

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Imu, "/imu_noisy", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan_noisy", self.scan_callback, 10)

        # ── State for statistics ──────────────────────────────
        self.imu_gyro_z_samples = []  # collect samples for statistics
        self.scan_count = 0
        self.outlier_count = 0

        # Print timer — summary every 5 seconds
        self.create_timer(5.0, self.print_summary)

        self.get_logger().info("SensorObserver started. Listening to sensors...")

    # ─────────────────────────────────────────────────────────
    # TODO 1a: Odometry callback
    # Extract x, y position and yaw from the odometry message.
    # Print them to the console.
    # Hint: position is in msg.pose.pose.position
    #       orientation is a quaternion in msg.pose.pose.orientation
    #       convert quaternion to yaw using:
    #           yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    # ─────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        # TODO: extract x, y, yaw and log them
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 1b: IMU callback
    # Extract angular_velocity.z (yaw rate) from the IMU message.
    # Store samples in self.imu_gyro_z_samples for statistics.
    # ─────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        # TODO: collect angular_velocity.z samples
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 1c: LiDAR callback
    # Count how many readings are outliers.
    # An outlier is: inf, NaN, or < range_min, or > range_max
    # Also find the minimum valid range (nearest obstacle).
    # ─────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        # TODO: count total rays and outlier rays
        pass

    # ─────────────────────────────────────────────────────────
    # TODO 1d: Print summary statistics
    # Every 5 seconds, print:
    #   - Mean and std dev of IMU gyro Z samples
    #   - Outlier percentage in LiDAR
    # Clear the sample buffer after printing.
    # ─────────────────────────────────────────────────────────
    def print_summary(self):
        # TODO: compute and print statistics
        # Hint: use sum()/len() for mean
        #       std = math.sqrt(sum((x-mean)**2 for x in samples) / len(samples))
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SensorObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
