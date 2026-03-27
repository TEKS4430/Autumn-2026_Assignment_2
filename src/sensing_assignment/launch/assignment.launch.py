#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    package_dir = get_package_share_directory("sensing_assignment")
    robot_description_path = os.path.join(
        package_dir, "resource", "TurtleBot3Burger.urdf"
    )

    turtlebot_driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {"robot_description": robot_description_path},
        ],
        output="screen",
    )

    motion_controller = Node(
        package="sensing_assignment",
        executable="motion_controller",
        name="motion_controller",
        output="screen",
    )
    noise_injector = Node(
        package="sensing_assignment",
        executable="noise_injector",
        name="noise_injector",
        output="screen",
    )
    task1_observer = Node(
        package="sensing_assignment",
        executable="task1_observer",
        name="task1_observer",
        output="screen",
    )
    task2_filter = Node(
        package="sensing_assignment",
        executable="task2_filter",
        name="task2_filter",
        output="screen",
    )
    task3_fusion = Node(
        package="sensing_assignment",
        executable="task3_fusion",
        name="task3_fusion",
        output="screen",
    )

    return LaunchDescription(
        [
            turtlebot_driver,
            motion_controller,
            noise_injector,
            task1_observer,
            task2_filter,
            task3_fusion,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=turtlebot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
