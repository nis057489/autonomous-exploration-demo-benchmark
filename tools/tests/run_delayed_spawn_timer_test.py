#!/usr/bin/env python3
"""Exercise the spawn timer with real ROS launch and a synthetic /clock.

Run in a sourced ROS environment with an unused ROS_DOMAIN_ID. No simulator
or robot nodes are started. Optional argument: navigation launch file path.
"""
import asyncio
import importlib.util
from pathlib import Path
import sys

import rclpy
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction
from launch.events import Shutdown
from launch_ros.actions import SetUseSimTime
from rosgraph_msgs.msg import Clock


async def check(launch_file):
    spec = importlib.util.spec_from_file_location("navigation_launch", launch_file)
    navigation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(navigation)
    rclpy.init()
    publisher_node = rclpy.create_node("spawn_test_clock")
    publisher = publisher_node.create_publisher(Clock, "/clock", 10)
    fired = []

    def record(context):
        now = navigation.get_ros_node(context).get_clock().now().nanoseconds / 1e9
        fired.append(now)
        return []

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetUseSimTime(True),
        OpaqueFunction(function=navigation._at_sim_time, kwargs={
            "spawn_time_s": 30.0, "actions": [OpaqueFunction(function=record)],
        }),
    ]))
    task = asyncio.create_task(service.run_async())

    async def clock(seconds, wall_seconds=0.8):
        # Repeat for DDS discovery, and leave the simulation clock paused
        # while real time advances. A wall-clock timer must not release us.
        stop = asyncio.get_running_loop().time() + wall_seconds
        while asyncio.get_running_loop().time() < stop:
            msg = Clock()
            msg.clock.sec = seconds
            publisher.publish(msg)
            await asyncio.sleep(0.05)

    try:
        await clock(0, 1.0)
        await clock(29)
        assert fired == [], f"Spawn fired before simulation time 30: {fired}"
        await clock(30)
        assert fired == [30.0], f"Spawn did not fire at simulation time 30: {fired}"
        await clock(60)
        assert fired == [30.0], f"Spawn fired more than once: {fired}"

        # Scheduling after the requested timestamp must execute immediately,
        # rather than adding another 30 seconds to the current simulation time.
        service.include_launch_description(LaunchDescription([
            OpaqueFunction(function=navigation._at_sim_time, kwargs={
                "spawn_time_s": 30.0, "actions": [OpaqueFunction(function=record)],
            }),
        ]))
        await clock(60)
        assert fired == [30.0, 60.0], f"Late scheduling added a delay: {fired}"
        print("PASS: no early spawn, spawn at sim 30, one shot, late launch immediate")
    finally:
        await service.emit_event_async(Shutdown(reason="test complete"))
        await task
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    default = (root / "simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation"
               / "launch/multi_robot_navigation_with_slam.launch.py")
    asyncio.run(check(Path(sys.argv[1]) if len(sys.argv) > 1 else default))
