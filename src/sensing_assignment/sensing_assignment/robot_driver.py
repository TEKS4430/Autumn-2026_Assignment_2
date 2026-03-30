import math
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# ── Physical constants (TurtleBot3 Burger) ───────────────────
WHEEL_DISTANCE = 0.160  # metres between wheel centres
MAX_SPEED = 6.67  # rad/s motor limit

# ── Simulated wheel asymmetry ─────────────────────────────────
# Real robots have slightly different wheel sizes due to manufacturing
# tolerances and uneven wear. This causes odometry to drift even when
# the robot is commanded to drive straight or in a perfect circle.
#
# Here the right wheel is modelled as 2.4% larger than spec.
# The motors still execute commands perfectly (Webots is ideal),
# but the odometry calculation uses these mismatched values,
# so the reported position drifts from the true position over time.
# This is exactly what happens with a real uncalibrated robot.
# ── Constants ─────────────────────────────────────────────────
WHEEL_DISTANCE = 0.160
WHEEL_RADIUS = 0.033  # nominal — used for motor commands
MAX_SPEED = 6.67

# Mismatched radii used only in odometry calculation
# Simulates the robot being uncalibrated — it thinks wheels are equal
# but they are slightly different, causing pose estimation to drift
ODOM_RADIUS_LEFT = 0.033
ODOM_RADIUS_RIGHT = 0.03380  # 2.4% larger than reality


class RobotDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # ── Motors ────────────────────────────────────────────
        self.__left_motor = self.__robot.getDevice("left wheel motor")
        self.__right_motor = self.__robot.getDevice("right wheel motor")
        self.__left_motor.setPosition(float("inf"))
        self.__right_motor.setPosition(float("inf"))
        self.__left_motor.setVelocity(0.0)
        self.__right_motor.setVelocity(0.0)

        # ── Wheel encoders ────────────────────────────────────
        self.__left_sensor = self.__robot.getDevice("left wheel sensor")
        self.__right_sensor = self.__robot.getDevice("right wheel sensor")
        self.__left_sensor.enable(self.__timestep)
        self.__right_sensor.enable(self.__timestep)

        # ── Odometry state ────────────────────────────────────
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0
        self.__prev_left = None
        self.__prev_right = None

        # ── Velocity commands ─────────────────────────────────
        self.__target_linear = 0.0
        self.__target_angular = 0.0

        # ── ROS2 interface ────────────────────────────────────
        # Create own node — WebotsNode does not expose the rclpy node
        # directly (see official plugin_example.py pattern)
        rclpy.init(args=None)
        self.__node = rclpy.create_node("robot_driver")

        self.__node.create_subscription(
            TwistStamped, "/cmd_vel", self.__cmd_vel_callback, 1
        )

        self.__odom_pub = self.__node.create_publisher(Odometry, "/odom", 10)

    def __cmd_vel_callback(self, msg):
        self.__target_linear = msg.twist.linear.x
        self.__target_angular = msg.twist.angular.z

    def step(self):
        # Spin once to process incoming /cmd_vel messages
        rclpy.spin_once(self.__node, timeout_sec=0)

        # ── Motor control (differential drive kinematics) ─────
        # Simulated motor asymmetry — left motor runs 1.5% faster than commanded.
        # This causes the robot to physically spiral rather than drive a perfect circle,
        # demonstrating real-world drift from motor imperfection.
        # Students observe the spiral via the Pen trail in Webots, and see the
        # accumulated position error in the /odom topic over time.
        left_speed = (
            (2 * self.__target_linear - self.__target_angular * WHEEL_DISTANCE)
            / (2 * WHEEL_RADIUS)
            * 1.015
        )
        right_speed = (
            2 * self.__target_linear + self.__target_angular * WHEEL_DISTANCE
        ) / (2 * WHEEL_RADIUS)

        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

        self.__left_motor.setVelocity(left_speed)
        self.__right_motor.setVelocity(right_speed)

        # ── Odometry from wheel encoders ──────────────────────
        # Uses mismatched wheel radii — this is the source of odometry drift.
        # The true motion is perfect (Webots), but the reported pose is wrong.
        left_pos = self.__left_sensor.getValue()
        right_pos = self.__right_sensor.getValue()

        if self.__prev_left is None:
            self.__prev_left = left_pos
            self.__prev_right = right_pos
            return

        # Arc length travelled by each wheel since last step
        dl = (left_pos - self.__prev_left) * ODOM_RADIUS_LEFT
        dr = (right_pos - self.__prev_right) * ODOM_RADIUS_RIGHT

        self.__prev_left = left_pos
        self.__prev_right = right_pos

        # Differential drive pose integration
        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / WHEEL_DISTANCE

        self.__x += dc * math.cos(self.__theta + dtheta / 2.0)
        self.__y += dc * math.sin(self.__theta + dtheta / 2.0)
        self.__theta += dtheta

        # ── Publish /odom ─────────────────────────────────────
        msg = Odometry()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = self.__x
        msg.pose.pose.position.y = self.__y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.__theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.__theta / 2.0)

        msg.twist.twist.linear.x = self.__target_linear
        msg.twist.twist.angular.z = self.__target_angular

        self.__odom_pub.publish(msg)
