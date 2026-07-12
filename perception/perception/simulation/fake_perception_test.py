#!/usr/bin/env python3
"""Synthetic end-to-end input and result checker for detect1 + tracking1."""

import math

import rclpy
from f110_msgs.msg import ObstacleArray, Wpnt, WpntArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class FakePerceptionTest(Node):
    def __init__(self):
        super().__init__("fake_perception_test")
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.wpnt_pub = self.create_publisher(WpntArray, "/global_waypoints", 10)
        self.frenet_odom_pub = self.create_publisher(
            Odometry, "/car_state/frenet/odom", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/car_state/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.raw_count = 0
        self.tracked_count = 0
        self.reported = False
        self.create_subscription(
            ObstacleArray, "/perception/detection/raw_obstacles", self.raw_cb, 10
        )
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.tracked_cb, 10)
        self.waypoints = self.make_waypoints()
        self.create_timer(0.025, self.publish_inputs)
        self.create_timer(1.0, self.publish_waypoints)
        self.create_timer(0.5, self.report)
        self.get_logger().info("Fake test started: one static obstacle is 2 m ahead")

    def make_waypoints(self):
        msg = WpntArray()
        radius = 5.0
        count = 100
        circumference = 2.0 * math.pi * radius
        for idx in range(count + 1):
            angle = 2.0 * math.pi * idx / count
            wpnt = Wpnt()
            wpnt.id = idx
            wpnt.s_m = circumference * idx / count
            wpnt.x_m = radius * math.cos(angle)
            wpnt.y_m = radius * math.sin(angle)
            wpnt.d_right = 2.0
            wpnt.d_left = 2.0
            msg.wpnts.append(wpnt)
        return msg

    def publish_waypoints(self):
        self.waypoints.header.stamp = self.get_clock().now().to_msg()
        self.waypoints.header.frame_id = "map"
        self.wpnt_pub.publish(self.waypoints)

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        self.publish_tf(stamp)
        self.publish_odom(stamp)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = -3.0 * math.pi / 4.0
        scan.angle_max = 3.0 * math.pi / 4.0
        scan.angle_increment = math.radians(0.25)
        scan.time_increment = 0.0
        scan.scan_time = 0.025
        scan.range_min = 0.05
        scan.range_max = 10.0
        count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        scan.ranges = [float("inf")] * count
        # A 0.30 m wide surface centered at x=2.0 in the laser frame.
        half_angle = math.atan2(0.15, 2.0)
        for idx in range(count):
            angle = scan.angle_min + idx * scan.angle_increment
            if abs(angle) <= half_angle:
                scan.ranges[idx] = 2.0 / max(math.cos(angle), 1e-6)
        self.scan_pub.publish(scan)

    def publish_tf(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "laser"
        transform.transform.translation.x = 5.0
        transform.transform.rotation.z = math.sin(math.pi / 4.0)
        transform.transform.rotation.w = math.cos(math.pi / 4.0)
        self.tf_broadcaster.sendTransform(transform)

    def publish_odom(self, stamp):
        frenet = Odometry()
        frenet.header.stamp = stamp
        frenet.header.frame_id = "map"
        frenet.pose.pose.position.x = 0.0
        self.frenet_odom_pub.publish(frenet)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = 5.0
        odom.pose.pose.orientation.z = math.sin(math.pi / 4.0)
        odom.pose.pose.orientation.w = math.cos(math.pi / 4.0)
        self.odom_pub.publish(odom)

    def raw_cb(self, msg):
        if msg.obstacles:
            self.raw_count += 1

    def tracked_cb(self, msg):
        if msg.obstacles:
            self.tracked_count += 1

    def report(self):
        if self.reported:
            return
        if self.raw_count and self.tracked_count:
            self.get_logger().info(
                "PASS: detection1 raw output=%d, tracking1 output=%d"
                % (self.raw_count, self.tracked_count)
            )
            self.reported = True


def main():
    rclpy.init()
    node = FakePerceptionTest()
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
