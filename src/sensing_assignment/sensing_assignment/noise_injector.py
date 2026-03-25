#!/usr/bin/env python3
"""
noise_injector.py  — PROVIDED, do not modify.

Subscribes to the raw sensor topics published by the Webots driver
and republishes them with realistic sensor errors added on top:

    /imu   → /imu_noisy
        - Gaussian noise on all axes
        - Constant gyroscope bias (drift) on Z axis
        - This simulates an uncalibrated IMU

    /scan  → /scan_noisy
        - Random outlier readings (spikes and dropouts)
        - Simulates dust, reflective surfaces, sensor glitches

Students work with /imu_noisy and /scan_noisy in their tasks.
The clean /imu and /scan are still available for comparison.

This models the lecture concept: raw sensor data is never perfect.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import numpy as np


# ── Noise parameters ─────────────────────────────────────────
# These are intentionally not documented to students —
# part of the assignment is to characterize the noise.

IMU_GYRO_NOISE_STD = 0.02  # rad/s — random noise on gyro
IMU_ACCEL_NOISE_STD = 0.05  # m/s² — random noise on accel
IMU_GYRO_BIAS_Z = 0.015  # rad/s — constant drift (students must find this)

LIDAR_OUTLIER_PROB = 0.03  # probability of a random spike per ray
LIDAR_DROPOUT_PROB = 0.02  # probability of a ray returning inf


class NoiseInjector(Node):

    def __init__(self):
        super().__init__("noise_injector")
        self.rng = np.random.default_rng(seed=42)  # reproducible noise

        # ── IMU ───────────────────────────────────────────────
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, "/imu_noisy", 10)

        # ── LiDAR ─────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan_noisy", 10)

        self.get_logger().info(
            "NoiseInjector running:\n"
            "  /imu   → /imu_noisy   (gyro bias + noise)\n"
            "  /scan  → /scan_noisy  (outliers + dropouts)"
        )

    def imu_callback(self, msg: Imu):
        noisy = Imu()
        noisy.header = msg.header

        # ── Add noise to linear acceleration ─────────────────
        noisy.linear_acceleration.x = msg.linear_acceleration.x + self.rng.normal(
            0, IMU_ACCEL_NOISE_STD
        )
        noisy.linear_acceleration.y = msg.linear_acceleration.y + self.rng.normal(
            0, IMU_ACCEL_NOISE_STD
        )
        noisy.linear_acceleration.z = msg.linear_acceleration.z + self.rng.normal(
            0, IMU_ACCEL_NOISE_STD
        )

        # ── Add noise + constant bias to angular velocity ─────
        # The bias on Z (yaw axis) causes heading to drift over time
        noisy.angular_velocity.x = msg.angular_velocity.x + self.rng.normal(
            0, IMU_GYRO_NOISE_STD
        )
        noisy.angular_velocity.y = msg.angular_velocity.y + self.rng.normal(
            0, IMU_GYRO_NOISE_STD
        )
        noisy.angular_velocity.z = (
            msg.angular_velocity.z
            + self.rng.normal(0, IMU_GYRO_NOISE_STD)
            + IMU_GYRO_BIAS_Z
        )  # ← constant bias = drift

        noisy.orientation = msg.orientation
        noisy.orientation_covariance = msg.orientation_covariance
        noisy.angular_velocity_covariance = msg.angular_velocity_covariance
        noisy.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_pub.publish(noisy)

    def scan_callback(self, msg: LaserScan):
        noisy = LaserScan()
        noisy.header = msg.header
        noisy.angle_min = msg.angle_min
        noisy.angle_max = msg.angle_max
        noisy.angle_increment = msg.angle_increment
        noisy.time_increment = msg.time_increment
        noisy.scan_time = msg.scan_time
        noisy.range_min = msg.range_min
        noisy.range_max = msg.range_max

        ranges = list(msg.ranges)
        for i in range(len(ranges)):
            r = self.rng.random()
            if r < LIDAR_DROPOUT_PROB:
                # Ray returns inf (missed return)
                ranges[i] = float("inf")
            elif r < LIDAR_DROPOUT_PROB + LIDAR_OUTLIER_PROB:
                # Random spike (false close reading)
                ranges[i] = self.rng.uniform(0.05, 0.15)
            # else: keep original value

        noisy.ranges = ranges
        noisy.intensities = msg.intensities

        self.scan_pub.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = NoiseInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
