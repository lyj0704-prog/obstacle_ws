#!/usr/bin/env python3
"""Print obstacle track IDs and classification stability once per second."""

import rclpy
from f110_msgs.msg import ObstacleArray
from rclpy.node import Node


class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__("obs_monitor")
        self.seen_ids = set()
        self.flips = 0
        self.last_flags = {}
        self.latest = []
        self.create_subscription(
            ObstacleArray, "/perception/obstacles", self.obstacle_cb, 10
        )
        self.create_timer(1.0, self.report)

    def obstacle_cb(self, msg):
        self.latest = [(obstacle.id, obstacle.is_static) for obstacle in msg.obstacles]
        for obstacle_id, is_static in self.latest:
            self.seen_ids.add(obstacle_id)
            previous = self.last_flags.get(obstacle_id)
            if previous is not None and previous != is_static:
                self.flips += 1
                self.get_logger().warn(
                    f"id {obstacle_id} classification changed: static={is_static}"
                )
            self.last_flags[obstacle_id] = is_static

    def report(self):
        current_ids = sorted(obstacle_id for obstacle_id, _ in self.latest)
        static_ids = sorted(
            obstacle_id for obstacle_id, is_static in self.latest if is_static
        )
        dynamic_ids = sorted(
            obstacle_id for obstacle_id, is_static in self.latest if not is_static
        )
        self.get_logger().info(
            f"tracks={len(self.latest)} ids={current_ids} "
            f"static={static_ids} dynamic/unclassified={dynamic_ids} | "
            f"unique_ids={len(self.seen_ids)} classification_flips={self.flips}"
        )


def main():
    rclpy.init()
    node = ObstacleMonitor()
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
