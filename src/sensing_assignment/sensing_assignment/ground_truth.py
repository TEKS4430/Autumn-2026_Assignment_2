#!/usr/bin/env python3
"""
ground_truth.py  — PROVIDED, do not modify.

Supervisor node that reads the TurtleBot3's TRUE position directly
from the Webots simulation engine (no sensor noise) and publishes it.

Students use /ground_truth_pose to evaluate how much their odometry
or fused estimate has drifted from reality.

Published topics:
    /ground_truth_pose  (geometry_msgs/PoseStamped)  — true robot pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math


class GroundTruthPublisher(Node):

    def __init__(self, robot):
        super().__init__("ground_truth_publisher")
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        # Get reference to TurtleBot3 via Supervisor API
        self.turtlebot = robot.getFromDef("TURTLEBOT")
        if self.turtlebot is None:
            self.get_logger().error(
                'Could not find robot DEF "TURTLEBOT" in the world. '
                "Make sure the TurtleBot3Burger node has DEF TURTLEBOT set."
            )

        # Publisher
        self.pub = self.create_publisher(PoseStamped, "/ground_truth_pose", 10)

        self.get_logger().info("GroundTruthPublisher started → /ground_truth_pose")

    def publish_ground_truth(self):
        if self.turtlebot is None:
            return

        # Read true position and orientation from Webots
        pos = self.turtlebot.getPosition()  # [x, y, z]
        rot = self.turtlebot.getOrientation()  # 3x3 rotation matrix (row-major)

        # Extract yaw from rotation matrix
        # rot = [r00, r01, r02, r10, r11, r12, r20, r21, r22]
        yaw = math.atan2(rot[3], rot[0])  # atan2(r10, r00)

        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion (rotation around Z only)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub.publish(msg)


def main(args=None):
    from controller import Supervisor

    rclpy.init(args=args)
    robot = Supervisor()
    node = GroundTruthPublisher(robot)
    timestep = node.timestep

    while robot.step(timestep) != -1:
        node.publish_ground_truth()
        rclpy.spin_once(node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
