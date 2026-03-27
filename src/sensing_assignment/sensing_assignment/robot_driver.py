import math
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

WHEEL_DISTANCE = 0.160
WHEEL_RADIUS = 0.033
MAX_SPEED = 6.67


class RobotDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Motors
        self.__left_motor = self.__robot.getDevice("left wheel motor")
        self.__right_motor = self.__robot.getDevice("right wheel motor")
        self.__left_motor.setPosition(float("inf"))
        self.__right_motor.setPosition(float("inf"))
        self.__left_motor.setVelocity(0.0)
        self.__right_motor.setVelocity(0.0)

        # Encoders
        self.__left_sensor = self.__robot.getDevice("left wheel sensor")
        self.__right_sensor = self.__robot.getDevice("right wheel sensor")
        self.__left_sensor.enable(self.__timestep)
        self.__right_sensor.enable(self.__timestep)

        # Odometry state
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0
        self.__prev_left = None
        self.__prev_right = None
        self.__target_linear = 0.0
        self.__target_angular = 0.0

        # Create own ROS node — per official plugin_example.py pattern
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
        # Spin to process callbacks — per official plugin_example.py pattern
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Motor control
        left_speed = (
            2 * self.__target_linear - self.__target_angular * WHEEL_DISTANCE
        ) / (2 * WHEEL_RADIUS)
        right_speed = (
            2 * self.__target_linear + self.__target_angular * WHEEL_DISTANCE
        ) / (2 * WHEEL_RADIUS)
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
        self.__left_motor.setVelocity(left_speed * 1.02)
        self.__right_motor.setVelocity(right_speed)

        # Odometry from encoders
        left_pos = self.__left_sensor.getValue()
        right_pos = self.__right_sensor.getValue()

        if self.__prev_left is None:
            self.__prev_left = left_pos
            self.__prev_right = right_pos
            return

        dl = (left_pos - self.__prev_left) * WHEEL_RADIUS
        dr = (right_pos - self.__prev_right) * WHEEL_RADIUS
        self.__prev_left = left_pos
        self.__prev_right = right_pos

        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / WHEEL_DISTANCE
        self.__x += dc * math.cos(self.__theta + dtheta / 2.0)
        self.__y += dc * math.sin(self.__theta + dtheta / 2.0)
        self.__theta += dtheta

        msg = Odometry()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.__x
        msg.pose.pose.position.y = self.__y
        msg.pose.pose.orientation.z = math.sin(self.__theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.__theta / 2.0)
        msg.twist.twist.linear.x = self.__target_linear
        msg.twist.twist.angular.z = self.__target_angular
        self.__odom_pub.publish(msg)
