#!/usr/bin/env python3
"""Apply a smooth, map-specific detour to a nominal global trajectory."""

import copy
import math

import rclpy
from f110_msgs.msg import WpntArray
from rclpy.node import Node


def wrapped_delta(value, center, length):
    return (value - center + 0.5 * length) % length - 0.5 * length


class StaticPathDetour(Node):
    def __init__(self):
        super().__init__("static_path_detour")
        self.declare_parameter("obstacle_s", 7.780)
        self.declare_parameter("offset", 0.40)
        self.declare_parameter("half_width", 1.50)
        self.obstacle_s = float(self.get_parameter("obstacle_s").value)
        self.offset = float(self.get_parameter("offset").value)
        self.half_width = float(self.get_parameter("half_width").value)

        self.publisher = self.create_publisher(WpntArray, "/global_waypoints", 10)
        self.create_subscription(
            WpntArray, "/global_waypoints_nominal", self.waypoints_cb, 10
        )
        self.get_logger().info(
            "Static detour enabled: obstacle_s=%.3f, offset=%.2f, half_width=%.2f"
            % (self.obstacle_s, self.offset, self.half_width)
        )

    def waypoints_cb(self, source):
        if len(source.wpnts) < 3:
            return
        result = copy.deepcopy(source)
        track_length = max(wp.s_m for wp in source.wpnts)
        offsets = []

        for src, dst in zip(source.wpnts, result.wpnts):
            ds = wrapped_delta(src.s_m, self.obstacle_s, track_length)
            if abs(ds) < self.half_width:
                weight = 0.5 * (1.0 + math.cos(math.pi * ds / self.half_width))
                lateral_offset = self.offset * weight
            else:
                lateral_offset = 0.0
            offsets.append(lateral_offset)

            nx = -math.sin(src.psi_rad)
            ny = math.cos(src.psi_rad)
            dst.x_m = src.x_m + lateral_offset * nx
            dst.y_m = src.y_m + lateral_offset * ny
            # Positive offset moves toward the original left boundary.
            dst.d_left = max(0.05, src.d_left - lateral_offset)
            dst.d_right = max(0.05, src.d_right + lateral_offset)

        # Recompute heading and curvature for the controller after moving points.
        count = len(result.wpnts)
        for idx, wp in enumerate(result.wpnts):
            prev_wp = result.wpnts[(idx - 1) % count]
            next_wp = result.wpnts[(idx + 1) % count]
            wp.psi_rad = math.atan2(next_wp.y_m - prev_wp.y_m, next_wp.x_m - prev_wp.x_m)

        for idx, wp in enumerate(result.wpnts):
            prev_wp = result.wpnts[(idx - 1) % count]
            next_wp = result.wpnts[(idx + 1) % count]
            ds = wrapped_delta(next_wp.s_m, prev_wp.s_m, track_length)
            dpsi = math.atan2(
                math.sin(next_wp.psi_rad - prev_wp.psi_rad),
                math.cos(next_wp.psi_rad - prev_wp.psi_rad),
            )
            wp.kappa_radpm = dpsi / max(abs(ds), 1e-6)

        result.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(result)


def main():
    rclpy.init()
    node = StaticPathDetour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
