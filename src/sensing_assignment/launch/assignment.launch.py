#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def generate_launch_description():

    package_dir = get_package_share_directory("sensing_assignment")

    robot_description_path = os.path.join(
        package_dir, "resource", "TurtleBot3Burger.urdf"
    )
    ros2_control_params = os.path.join(package_dir, "resource", "ros2control.yml")

    # Minimal robot_state_publisher — driver fills it in via
    # set_robot_state_publisher: True
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )

    # Static transform base_link → base_footprint
    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    # TurtleBot3 driver — connects to Webots and sets up all topics
    turtlebot_driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": True,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=[
            ("/diffdrive_controller/cmd_vel", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ],
        respawn=True,
        output="screen",
    )

    # Spawn controllers — use WaitForControllerConnection so they
    # start only after the driver is ready
    diffdrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller", "--controller-manager-timeout", "50"],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "50"],
        output="screen",
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=[diffdrive_spawner, joint_state_spawner],
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
            robot_state_publisher,
            footprint_publisher,
            turtlebot_driver,
            waiting_nodes,
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
