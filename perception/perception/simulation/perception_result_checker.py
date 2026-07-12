#!/usr/bin/env python3
"""Report whether detection and tracking produce obstacles from live inputs."""

import rclpy
from f110_msgs.msg import ObstacleArray
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PerceptionResultChecker(Node):
    def __init__(self):
        super().__init__("perception_result_checker")
        self.raw_count = 0
        self.tracked_count = 0
        self.static_count = 0
        self.scan_count = 0
        self.nearest_range = float("inf")
        self.reported = False
        self.create_subscription(
            ObstacleArray, "/perception/detection/raw_obstacles", self.raw_cb, 10
        )
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.tracked_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_timer(1.0, self.report)

    def raw_cb(self, msg):
        if msg.obstacles:
            self.raw_count += 1

    def tracked_cb(self, msg):
        if msg.obstacles:
            self.tracked_count += 1
            self.static_count += sum(1 for obstacle in msg.obstacles if obstacle.is_static)

    def scan_cb(self, msg):
        self.scan_count += 1
        valid = [value for value in msg.ranges if msg.range_min < value < msg.range_max]
        if valid:
            self.nearest_range = min(valid)

    def report(self):
        if self.reported:
            return
        if self.raw_count and self.tracked_count and self.static_count:
            self.get_logger().info(
                "PASS: map obstacle detected and tracked as static "
                "(raw=%d, tracked=%d, static observations=%d)"
                % (self.raw_count, self.tracked_count, self.static_count)
            )
            self.reported = True
        else:
            self.get_logger().info(
                "WAITING: scans=%d nearest=%.2fm, raw=%d, tracked=%d, static=%d"
                % (
                    self.scan_count,
                    self.nearest_range,
                    self.raw_count,
                    self.tracked_count,
                    self.static_count,
                )
            )


def main():
    rclpy.init()
    node = PerceptionResultChecker()
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
