#!/usr/bin/env python3
"""
assignment.launch.py

Launches all nodes for the sensing assignment.

What gets launched:
    1. WebotsController for TurtleBot3  — bridges robot ↔ ROS2 topics
    2. WebotsController for Supervisor  — provides ground truth pose
    3. noise_injector    (PROVIDED)     — adds drift/noise to sensors
    4. motion_controller (PROVIDED)     — waits for /start_driving service
    5. task1_observer    (STUDENT)      — sensor observation
    6. task2_filter      (STUDENT)      — sensor filtering
    7. task3_fusion      (STUDENT)      — pose fusion

Usage:
    ros2 launch sensing_assignment assignment.launch.py

Then start the robot driving:
    ros2 service call /start_driving std_srvs/srv/Trigger {}
"""

import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():

    package_dir = get_package_share_directory("sensing_assignment")
    robot_urdf = os.path.join(package_dir, "resource", "turtlebot_webots.urdf")

    # ── Webots host (set via WEBOTS_HOST env var) ─────────────
    webots_host = os.environ.get("WEBOTS_HOST", "localhost")

    # ── 1. TurtleBot3 controller ──────────────────────────────
    # Bridges Webots TurtleBot3 robot ↔ ROS2 topics:
    #   /scan, /imu, /odom, /camera/image_raw, /camera/depth/image_raw
    turtlebot_driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {"robot_description": robot_urdf},
            {"use_sim_time": True},
        ],
        env={
            "WEBOTS_CONTROLLER_URL": f"tcp://{webots_host}:1234/TurtleBot3Burger",
        },
    )

    # ── 2. Supervisor controller ──────────────────────────────
    # Reads true robot position from Webots and publishes /ground_truth_pose
    supervisor_driver = WebotsController(
        robot_name="supervisor",
        parameters=[
            {"use_sim_time": True},
        ],
        env={
            "WEBOTS_CONTROLLER_URL": f"tcp://{webots_host}:1234/supervisor",
        },
    )

    # ── 3. Noise injector (PROVIDED) ──────────────────────────
    noise_injector = Node(
        package="sensing_assignment",
        executable="noise_injector",
        name="noise_injector",
        output="screen",
    )

    # ── 4. Motion controller (PROVIDED) ───────────────────────
    motion_controller = Node(
        package="sensing_assignment",
        executable="motion_controller",
        name="motion_controller",
        output="screen",
    )

    # ── 5. Task 1 — sensor observer (STUDENT) ─────────────────
    task1_observer = Node(
        package="sensing_assignment",
        executable="task1_observer",
        name="task1_observer",
        output="screen",
    )

    # ── 6. Task 2 — sensor filter (STUDENT) ───────────────────
    task2_filter = Node(
        package="sensing_assignment",
        executable="task2_filter",
        name="task2_filter",
        output="screen",
    )

    # ── 7. Task 3 — pose fusion (STUDENT) ─────────────────────
    task3_fusion = Node(
        package="sensing_assignment",
        executable="task3_fusion",
        name="task3_fusion",
        output="screen",
    )

    return LaunchDescription(
        [
            # Webots controllers first
            turtlebot_driver,
            supervisor_driver,
            # Provided nodes
            noise_injector,
            motion_controller,
            # Student nodes
            # task1_observer,
            # task2_filter,
            # task3_fusion,
            # Shutdown all nodes when Webots closes
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=turtlebot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
