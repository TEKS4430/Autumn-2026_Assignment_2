#!/usr/bin/env python3
"""
motion_controller.py  — PROVIDED, do not modify.

Controls the TurtleBot3 to drive in a circle using direct motor
velocity commands. Also reads wheel encoders to compute a simple
dead-reckoning position estimate and prints it to the console,
so students can observe how odometry drifts over time.

This node runs as a Webots robot controller (WebotsController),
meaning it has direct access to Webots devices.

Usage:
    This node starts but waits for the 'start_driving' ROS2 service.
    Students trigger it with:
        ros2 service call /start_driving std_srvs/srv/Trigger {}
    Stop with:
        ros2 service call /stop_driving std_srvs/srv/Trigger {}
"""
import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point


# Webots robot controller interface
from vehicle import Driver  # only available inside WebotsController context


# ── Robot physical parameters (TurtleBot3 Burger) ────────────
WHEEL_RADIUS = 0.033  # meters
WHEEL_DISTANCE = 0.160  # meters between wheel centers
MAX_VELOCITY = 6.67  # rad/s (motor limit)

# ── Circular motion parameters ────────────────────────────────
# Slightly different speeds on each wheel → robot drives in a circle.
# The small asymmetry also models real-world motor imperfection.
LEFT_SPEED = 2.0  # rad/s
RIGHT_SPEED = 2.2  # rad/s  ← intentionally different → drift


class MotionController(Node):

    def __init__(self, robot):
        super().__init__("motion_controller")
        self.robot = robot
        self.driving = False

        # ── Webots devices ────────────────────────────────────
        timestep = int(robot.getBasicTimeStep())
        self.timestep = timestep

        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.left_sensor = robot.getDevice("left wheel sensor")
        self.right_sensor = robot.getDevice("right wheel sensor")

        # Motors: set to velocity mode (position = infinity)
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Enable wheel encoders
        self.left_sensor.enable(timestep)
        self.right_sensor.enable(timestep)

        # ── Dead-reckoning state ──────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = 0.0
        self.prev_right = 0.0
        self.first_reading = True

        # ── ROS2 services ─────────────────────────────────────
        self.start_srv = self.create_service(
            Trigger, "start_driving", self.start_callback
        )
        self.stop_srv = self.create_service(Trigger, "stop_driving", self.stop_callback)

        # ── Publisher: dead-reckoning pose ────────────────────
        self.pose_pub = self.create_publisher(Point, "/odom_deadreck", 10)

        # ── Console print timer (every 2 seconds) ─────────────
        self.print_timer = self.create_timer(2.0, self.print_position)

        self.get_logger().info(
            "MotionController ready.\n"
            "  Start: ros2 service call /start_driving std_srvs/srv/Trigger {}\n"
            "  Stop:  ros2 service call /stop_driving  std_srvs/srv/Trigger {}"
        )

    # ── Service callbacks ─────────────────────────────────────

    def start_callback(self, request, response):
        self.driving = True
        self.left_motor.setVelocity(LEFT_SPEED)
        self.right_motor.setVelocity(RIGHT_SPEED)
        self.get_logger().info(
            f"Driving started. Left={LEFT_SPEED}, Right={RIGHT_SPEED} rad/s"
        )
        response.success = True
        response.message = "Robot is now driving in a circle."
        return response

    def stop_callback(self, request, response):
        self.driving = False
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.get_logger().info("Driving stopped.")
        response.success = True
        response.message = "Robot stopped."
        return response

    # ── Dead-reckoning update (called every Webots timestep) ──

    def update_odometry(self):
        left_pos = self.left_sensor.getValue()
        right_pos = self.right_sensor.getValue()

        if self.first_reading:
            self.prev_left = left_pos
            self.prev_right = right_pos
            self.first_reading = False
            return

        # Incremental encoder deltas (radians)
        d_left = (left_pos - self.prev_left) * WHEEL_RADIUS
        d_right = (right_pos - self.prev_right) * WHEEL_RADIUS
        self.prev_left = left_pos
        self.prev_right = right_pos

        # Differential drive kinematics
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / WHEEL_DISTANCE

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Publish dead-reckoning position
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.theta
        self.pose_pub.publish(msg)

    def print_position(self):
        """Print current dead-reckoning position to console."""
        self.get_logger().info(
            f"[Dead reckoning]  x={self.x:+.3f}m  y={self.y:+.3f}m  "
            f"θ={math.degrees(self.theta):+.1f}°"
        )


def main(args=None):
    # WebotsController context: robot object is passed automatically
    import sys
    from controller import Robot

    rclpy.init(args=args)
    robot = Robot()
    node = MotionController(robot)

    timestep = node.timestep

    # Main loop: alternate between Webots step and ROS2 spin
    while robot.step(timestep) != -1:
        node.update_odometry()
        rclpy.spin_once(node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
