#!/usr/bin/env python3
"""
Replay a precomputed capacity-vs-time schedule onto every DDIL link.

  link_schedule.json (tools/gen_link_schedule.py, generated BEFORE the run)
       |
       v
  bandwidth_kbps param set on every /ddil_proxy_{X}_from_{Y}
       |
       v
  each proxy rebuilds its TokenBucket and resizes its queue budget
  (ddil_proxy_node.cpp on_param_change) -- no C++ change needed here

This node is deliberately a dumb replayer: all the randomness and all the
policy live in the generator, which runs before the stack comes up and writes
its trace to disk. That split is what makes a run auditable -- the schedule is
a pre-run *input* you can diff between two arms to prove they saw identical
conditions, not a side effect of whatever this process happened to do.

FAIRNESS. This node writes to the link and nothing else. No encoder, scheduler,
or fusion node subscribes to it or to ddil_stats. Letting the vxch encoder see
capacity coming would be encoder backpressure -- an advantage the baseline
transport structurally cannot have, which would make the comparison meaningless.

The capacity timeline is not logged separately: every proxy already republishes
its live bandwidth_kbps on ~/ddil_stats at 5 Hz, and launch.sh bags that, so the
applied capacity and the response to it land in one bag on one clock.
"""

import json

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

PROXY_PREFIX = "ddil_proxy_"


class LinkScheduler(Node):
    def __init__(self):
        super().__init__("link_scheduler")

        schedule_path = self.declare_parameter("schedule_path", "").value
        # Number of (robot, peer) downlinks expected to exist: N*(N-1) for N
        # robots. We refuse to start the clock until they are all present --
        # see _poll_for_proxies.
        self.expected_links = int(self.declare_parameter("expected_links", 0).value)
        # How long to wait for that full set before giving up and running
        # anyway (with a loud warning). Never silently, and never forever.
        self.discovery_timeout_s = float(
            self.declare_parameter("discovery_timeout_s", 60.0).value)

        if not schedule_path:
            raise RuntimeError("link_scheduler: schedule_path parameter is required")

        with open(schedule_path) as f:
            self.schedule = json.load(f)
        self.segments = self.schedule.get("segments", [])
        if not self.segments:
            raise RuntimeError(f"link_scheduler: no segments in {schedule_path}")

        self.get_logger().info(
            f"loaded {schedule_path}: profile={self.schedule.get('profile')} "
            f"seed={self.schedule.get('seed')} "
            f"segments={len(self.segments)} "
            f"duration={self.schedule.get('duration_s')}s")

        # NOT `self.clients`: rclpy.node.Node already defines a read-only
        # `clients` property, and assigning over it raises at construction.
        self.param_clients = {}   # fully-qualified node name -> SetParameters client
        self.next_index = 0
        self.t0 = None          # run clock origin, set once all links are up
        self.discovery_started = self.get_clock().now()

        self.discovery_timer = self.create_timer(1.0, self._poll_for_proxies)
        # Re-scan periodically even after starting, so a proxy that restarts
        # mid-run gets picked up and re-set rather than sitting at its
        # launch-time default for the remainder (a silent per-link asymmetry).
        self.apply_timer = self.create_timer(0.2, self._tick)

    # -- link discovery ------------------------------------------------------

    def _discover(self):
        """Find every ddil_proxy_* node and make a SetParameters client for it."""
        found = 0
        for name, namespace in self.get_node_names_and_namespaces():
            if not name.startswith(PROXY_PREFIX):
                continue
            found += 1
            fq = f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            if fq not in self.param_clients:
                self.param_clients[fq] = self.create_client(
                    SetParameters, f"{fq}/set_parameters")
                self.get_logger().info(f"discovered link {fq}")
        return found

    def _poll_for_proxies(self):
        """Hold the run clock until every expected link is present.

        Starting the schedule with links still missing would leave those links
        at their launch-time bandwidth for the first stretch of the run while
        the others follow the schedule. That is not a small error: it is two
        different capacity conditions inside one run, and nothing downstream
        would show it as anything other than noise.
        """
        found = self._discover()
        if self.t0 is not None:
            return  # already running; _discover above still picks up restarts

        waited = (self.get_clock().now() - self.discovery_started).nanoseconds / 1e9
        if self.expected_links > 0 and found < self.expected_links:
            if waited < self.discovery_timeout_s:
                self.get_logger().info(
                    f"waiting for DDIL links: {found}/{self.expected_links} "
                    f"({waited:.0f}s/{self.discovery_timeout_s:.0f}s)",
                    throttle_duration_sec=5.0)
                return
            self.get_logger().error(
                f"only {found}/{self.expected_links} DDIL links found after "
                f"{waited:.0f}s -- starting the schedule anyway. The missing "
                f"link(s) will sit at their launch-time bandwidth, so THIS RUN "
                f"IS NOT COMPARABLE to one where all links were shaped.")
        elif found == 0:
            if waited < self.discovery_timeout_s:
                return
            self.get_logger().error(
                "no ddil_proxy_* nodes found -- schedule will have no effect.")

        self.t0 = self.get_clock().now()
        self.get_logger().info(
            f"schedule started, driving {found} link(s)")

    # -- schedule replay -----------------------------------------------------

    def _tick(self):
        if self.t0 is None:
            return
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9

        # Apply every segment whose time has passed. The loop (rather than a
        # single step) matters if the executor is ever starved past a short
        # segment: we still land on the correct current capacity instead of
        # walking the backlog one tick at a time.
        applied = None
        while (self.next_index < len(self.segments)
               and self.segments[self.next_index]["t"] <= elapsed):
            applied = self.segments[self.next_index]
            self.next_index += 1

        if applied is not None:
            self._apply(applied, elapsed)

    def _apply(self, segment, elapsed):
        kbps = float(segment["kbps"])
        value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=kbps)
        req = SetParameters.Request(
            parameters=[ParameterMsg(name="bandwidth_kbps", value=value)])

        sent = 0
        for fq, client in self.param_clients.items():
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"{fq} parameter service not ready -- link left at its "
                    f"previous bandwidth for now")
                continue
            client.call_async(req)
            sent += 1

        self.get_logger().info(
            f"t={elapsed:7.1f}s  bandwidth_kbps -> {kbps:.0f} "
            f"[{segment.get('state', '?')}] on {sent} link(s)")


def main():
    rclpy.init()
    node = LinkScheduler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch.sh's cleanup() SIGTERMs the whole stack at the end of every
        # run, so without this every single run ends with a Python traceback in
        # ros_logs/ that looks like the scheduler crashed.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
