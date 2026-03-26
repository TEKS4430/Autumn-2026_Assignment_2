#!/usr/bin/env python3
"""
ground_truth.py  — PROVIDED, do not modify.

Runs as a WebotsController for the 'supervisor' robot in Webots.
Uses the Webots Supervisor API to read the TurtleBot3's TRUE position
(no sensor noise) and publishes it to /ground_truth_pose.

Students compare their /pose_estimate against /ground_truth_pose
to measure how much drift their fusion corrects.

Published topics:
    /ground_truth_pose  (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class GroundTruthPublisher(Node):

    def __init__(self, robot):
        super().__init__("ground_truth_publisher")
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        # Find TurtleBot3 in the scene via its DEF name
        # Make sure TurtleBot3Burger has DEF TURTLEBOT in the .wbt file
        self.turtlebot = robot.getFromDef("TURTLEBOT")
        if self.turtlebot is None:
            self.get_logger().error(
                'DEF "TURTLEBOT" not found in Webots world.\n'
                "Add DEF TURTLEBOT to your TurtleBot3Burger node in the .wbt file."
            )
        else:
            self.get_logger().info("GroundTruthPublisher connected to TURTLEBOT.")

        self.pub = self.create_publisher(PoseStamped, "/ground_truth_pose", 10)

    def step(self):
        """Called every Webots timestep from the main loop."""
        if self.turtlebot is None:
            return

        pos = self.turtlebot.getPosition()  # [x, y, z] in world frame
        rot = self.turtlebot.getOrientation()  # 3x3 rotation matrix, row-major

        # Extract yaw from rotation matrix
        # For a rotation around Z: yaw = atan2(R[1][0], R[0][0])
        # In row-major flat list: R[1][0] = rot[3], R[0][0] = rot[0]
        yaw = math.atan2(rot[3], rot[0])

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub.publish(msg)


def main(args=None):
    # Must import here — only available inside WebotsController process
    from controller import Supervisor

    rclpy.init(args=args)
    robot = Supervisor()
    node = GroundTruthPublisher(robot)
    timestep = node.timestep

    while robot.step(timestep) != -1:
        node.step()
        rclpy.spin_once(node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
