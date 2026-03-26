#!/usr/bin/env python3
"""
motion_controller.py  — PROVIDED, do not modify.

Drives the TurtleBot3 in a circle by publishing to /cmd_vel.
Uses a slightly different linear/angular ratio to intentionally
introduce drift — the robot spirals rather than making a perfect circle.

This runs as a regular ROS2 node (no direct Webots access needed).
Motor commands go through /cmd_vel → TurtleBot3 driver → Webots motors.

Usage:
    Start driving:
        ros2 service call /start_driving std_srvs/srv/Trigger {}
    Stop:
        ros2 service call /stop_driving std_srvs/srv/Trigger {}
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

# ── Circular motion parameters ────────────────────────────────
# These produce a slow circle. The slight imprecision in angular
# velocity means the robot won't return exactly to its start —
# demonstrating odometric drift.
LINEAR_SPEED = 0.15  # m/s  — forward speed
ANGULAR_SPEED = 0.4  # rad/s — turning rate
# Theoretical circle radius = LINEAR_SPEED / ANGULAR_SPEED = 0.375m


class MotionController(Node):

    def __init__(self):
        super().__init__("motion_controller")
        self.driving = False

        # Publisher — sends velocity commands to the robot
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Services — students call these to start/stop
        self.create_service(Trigger, "start_driving", self.start_callback)
        self.create_service(Trigger, "stop_driving", self.stop_callback)

        # Timer — publishes cmd_vel at 10 Hz while driving
        self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info(
            "\n"
            "  MotionController ready.\n"
            "  Start the robot:\n"
            "    ros2 service call /start_driving std_srvs/srv/Trigger {}\n"
            "  Stop the robot:\n"
            "    ros2 service call /stop_driving std_srvs/srv/Trigger {}\n"
        )

    def start_callback(self, request, response):
        self.driving = True
        self.get_logger().info(
            f"Driving started — linear={LINEAR_SPEED} m/s, "
            f"angular={ANGULAR_SPEED} rad/s"
        )
        response.success = True
        response.message = "Robot is driving in a circle."
        return response

    def stop_callback(self, request, response):
        self.driving = False
        self.cmd_pub.publish(TwistStamped())
        self.get_logger().info("Robot stopped.")
        response.success = True
        response.message = "Robot stopped."
        return response

    def publish_cmd(self):
        if not self.driving:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = LINEAR_SPEED
        msg.twist.angular.z = ANGULAR_SPEED
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
