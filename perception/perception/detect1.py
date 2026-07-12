#!/usr/bin/env python3
"""
LiDAR 기반 장애물 검출 노드 (Adaptive Breakpoint + 회전 사각형 피팅).

Hokuyo UST-10LX (40Hz, 270°, 0.25°, 1081포인트) 대응 수정 사항:

1. [Hz] 타이머 폴링(10Hz, 스캔 4장 중 3장 폐기) -> 스캔 콜백 구동으로 변경.
   detect.rate(기본 40.0)는 최대 처리율 상한으로만 동작 (CPU 부족 시 낮추면
   자동으로 스캔을 건너뜀). tracking의 dt는 stamp 기반이라 자동 추종.
2. [모션 왜곡] 스캔 1회전에 ~25ms가 걸리는 동안 자차가 움직여 포인트가 번지는
   문제를 보정(de-skew). 스캔 시작/종료 시각의 TF 두 개를 조회해 포인트별
   측정 시각(t0 + i*time_increment)에 맞춰 pose를 선형/각도 보간 후 변환.
   - 종료 시각 TF가 아직 없으면 과거 구간(t0-duration ~ t0)으로 등속 외삽
   - 그것도 실패하면 단일 TF로 폴백 (기존 동작)
   - detect.deskew 파라미터로 on/off
3. [CPU] 40Hz 처리를 위해 ABD 클러스터링 포인트 루프를 numpy로 벡터화
   (1081포인트 x 40Hz 순수 Python 루프 제거).
4. RViz 마커는 detect.viz_rate(기본 10Hz)로만 publish (RViz 부하 절감).
   장애물 메시지는 매 스캔 publish.

(이전 버전에서 이미 반영된 사항: ABD d_max에 센서 range 사용, 유효 range 필터,
 거리 적응형 최소 포인트 수, 인접 클러스터 병합, track_length 단일화)
"""

import math
import time
from bisect import bisect_left

import numpy as np
import rclpy
from f110_msgs.msg import Obstacle as ObstacleMessage
from f110_msgs.msg import ObstacleArray, WpntArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .frenet_utils import (
    SimpleFrenetConverter,
    normalize_angle,
    normalize_s,
    quaternion_to_yaw,
)


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class DetectedObstacle:
    def __init__(self, x, y, size, theta):
        self.center_x = x
        self.center_y = y
        self.size = size
        self.id = None
        self.theta = theta


class Detect(Node):
    def __init__(self):
        super().__init__("detect")

        self.declare_parameter("measure", False)
        self.declare_parameter("from_bag", False)
        self.declare_parameter("detect.rate", 40.0)  # 최대 처리율 (UST-10LX 스캔 주기)
        self.declare_parameter("detect.viz_rate", 10.0)
        self.declare_parameter("detect.deskew", True)
        self.declare_parameter("detect.lambda", 10.0)
        self.declare_parameter("detect.sigma", 0.03)
        self.declare_parameter("detect.min_2_points_dist", 0.05)
        self.declare_parameter("detect.min_obs_size", 10)
        self.declare_parameter("detect.min_points_floor", 3)
        self.declare_parameter("detect.min_size_ref_dist", 3.0)
        self.declare_parameter("detect.cluster_merge_dist", 0.35)
        self.declare_parameter("detect.max_obs_size", 0.5)
        self.declare_parameter("detect.max_viewing_distance", 9.0)
        self.declare_parameter("detect.boundaries_inflation", 0.15)
        self.declare_parameter("map_frame", "map")

        self.measuring = bool(self.get_parameter("measure").value)
        self.rate = float(self.get_parameter("detect.rate").value)
        self.viz_rate = float(self.get_parameter("detect.viz_rate").value)
        self.deskew = bool(self.get_parameter("detect.deskew").value)
        self.lambda_angle = math.radians(float(self.get_parameter("detect.lambda").value))
        self.sigma = float(self.get_parameter("detect.sigma").value)
        self.min_2_points_dist = float(self.get_parameter("detect.min_2_points_dist").value)
        self.min_obs_size = int(self.get_parameter("detect.min_obs_size").value)
        self.min_points_floor = int(self.get_parameter("detect.min_points_floor").value)
        self.min_size_ref_dist = float(self.get_parameter("detect.min_size_ref_dist").value)
        self.cluster_merge_dist = float(self.get_parameter("detect.cluster_merge_dist").value)
        self.max_obs_size = float(self.get_parameter("detect.max_obs_size").value)
        self.max_viewing_distance = float(self.get_parameter("detect.max_viewing_distance").value)
        self.boundaries_inflation = float(self.get_parameter("detect.boundaries_inflation").value)
        self.map_frame = self.get_parameter("map_frame").value

        self.converter = None
        self.waypoints = None
        self.biggest_d = None
        self.smallest_d = None
        self.s_array = None
        self.d_right_array = None
        self.d_left_array = None
        self.track_length = None
        self.car_s = 0.0
        self.current_stamp = None
        self.detected_obstacles = []
        self.last_process_time = -1.0
        self.last_viz_time = -1.0
        self.deskew_warned = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 스캔 콜백 구동 (타이머 폴링 제거)
        self.create_subscription(LaserScan, "/scan", self.laser_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.path_cb, 10)
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.car_state_cb, 10)

        self.breakpoints_markers_pub = self.create_publisher(
            MarkerArray, "/perception/breakpoints_markers", 10
        )
        self.boundaries_pub = self.create_publisher(Marker, "/perception/detect_bound", 10)
        self.obstacles_msg_pub = self.create_publisher(
            ObstacleArray, "/perception/detection/raw_obstacles", 10
        )
        self.obstacles_marker_pub = self.create_publisher(
            MarkerArray, "/perception/obstacles_markers_new", 10
        )
        if self.measuring:
            self.latency_pub = self.create_publisher(
                Float32, "/perception/detection/latency", 10
            )

        self.get_logger().info(
            "[Opponent Detection]: ROS2 node ready "
            "(scan-driven, deskew=%s)" % self.deskew
        )

    # ------------------------------------------------------------------ callbacks

    def laser_cb(self, msg):
        if self.converter is None or self.track_length is None:
            return

        # 처리율 상한 (CPU 여유 없으면 detect.rate를 낮춰 스캔을 건너뛰게 함)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_process_time > 0.0 and (now - self.last_process_time) < 0.9 / self.rate:
            return
        self.last_process_time = now

        if self.measuring:
            start_time = time.perf_counter()

        self.current_stamp = msg.header.stamp
        objects = self.scans_to_obs_point_clouds(msg)
        current_obstacles = self.obs_point_clouds_to_obs_array(objects)
        self.check_obstacles(current_obstacles)
        self.publish_obstacles_message()

        # 마커는 viz_rate로만 (RViz 부하 절감)
        if now - self.last_viz_time >= 1.0 / self.viz_rate:
            self.last_viz_time = now
            self.publish_breakpoint_markers(objects)
            self.publish_obstacles_markers()

        if self.measuring:
            latency = Float32()
            latency.data = float(time.perf_counter() - start_time)
            self.latency_pub.publish(latency)

    def path_cb(self, data):
        if not data.wpnts:
            return

        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        if self.converter is None:
            self.converter = SimpleFrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
            self.get_logger().info("[Opponent Detection]: initialized SimpleFrenetConverter")

        self.s_array = []
        self.d_right_array = []
        self.d_left_array = []
        points = []

        for waypoint in data.wpnts:
            self.s_array.append(waypoint.s_m)
            self.d_right_array.append(waypoint.d_right - self.boundaries_inflation)
            self.d_left_array.append(waypoint.d_left - self.boundaries_inflation)
            right = self.converter.get_cartesian(
                waypoint.s_m, -waypoint.d_right + self.boundaries_inflation
            )
            left = self.converter.get_cartesian(
                waypoint.s_m, waypoint.d_left - self.boundaries_inflation
            )
            points.append(Point(x=float(right[0]), y=float(right[1]), z=0.0))
            points.append(Point(x=float(left[0]), y=float(left[1]), z=0.0))

        self.smallest_d = min(self.d_right_array + self.d_left_array)
        self.biggest_d = max(self.d_right_array + self.d_left_array)
        self.track_length = self.converter.track_length
        self.publish_boundary(points)

    def car_state_cb(self, data):
        self.car_s = data.pose.pose.position.x

    # ------------------------------------------------------------------ helpers

    def publish_boundary(self, points):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.points = points
        self.boundaries_pub.publish(marker)

    def clear_marker_array(self):
        array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        array.markers.append(marker)
        return array

    def laser_point_on_track(self, s, d):
        if self.track_length is None:
            return False
        if normalize_s(s - self.car_s, self.track_length) > self.max_viewing_distance:
            return False
        if abs(d) >= self.biggest_d:
            return False
        if abs(d) <= self.smallest_d:
            return True
        idx = bisect_left(self.s_array, s)
        if idx:
            idx -= 1
        if d <= -self.d_right_array[idx] or d >= self.d_left_array[idx]:
            return False
        return True

    # ------------------------------------------------------------------ TF / de-skew

    def lookup_pose(self, frame, tf_time):
        transform = self.tf_buffer.lookup_transform(self.map_frame, frame, tf_time)
        translation = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        return np.array([translation.x, translation.y]), yaw

    def get_pose_pair(self, frame, t0, duration):
        """
        스캔 시작/종료 시각의 (위치, yaw) 쌍을 반환.

        우선순위:
        1) (t0, t0+duration) 정상 보간
        2) 종료 시각 TF가 아직 없으면 (t0-duration, t0)로 등속 외삽
        3) 시작 시각 TF도 없으면 latest 단일 pose (de-skew 없음)
        실패 시 None 반환.
        """
        try:
            pose0 = self.lookup_pose(frame, t0)
        except TransformException:
            try:
                pose_latest = self.lookup_pose(frame, Time())
            except TransformException as exc:
                self.get_logger().warn(
                    f"[Opponent Detection]: transform "
                    f"{self.map_frame}->{frame} unavailable: {exc}",
                    throttle_duration_sec=1.0,
                )
                return None
            return pose_latest, pose_latest

        if not self.deskew or duration <= 1e-4:
            return pose0, pose0

        try:
            pose1 = self.lookup_pose(frame, t0 + Duration(seconds=duration))
            return pose0, pose1
        except TransformException:
            pass

        try:
            # 종료 시각 TF 미도착 -> 직전 구간으로 등속 외삽
            pose_prev = self.lookup_pose(frame, t0 - Duration(seconds=duration))
            trans1 = pose0[0] + (pose0[0] - pose_prev[0])
            yaw1 = pose0[1] + normalize_angle(pose0[1] - pose_prev[1])
            return pose0, (trans1, yaw1)
        except TransformException:
            if not self.deskew_warned:
                self.get_logger().warn(
                    "[Opponent Detection]: de-skew TF pair unavailable, "
                    "falling back to single transform",
                )
                self.deskew_warned = True
            return pose0, pose0

    def transform_scan_to_map(self, msg):
        """
        스캔을 map 프레임으로 변환 (+ 모션 왜곡 보정).

        반환: (N,2) map 좌표 배열, (N,) 센서 range 배열. 실패 시 (None, None).
        """
        n_total = len(msg.ranges)
        if n_total == 0:
            return None, None

        # 포인트별 측정 시각 오프셋
        time_inc = float(msg.time_increment)
        if time_inc <= 0.0 and msg.scan_time > 0.0:
            time_inc = float(msg.scan_time) / float(n_total)
        duration = time_inc * (n_total - 1)

        pose_pair = self.get_pose_pair(
            msg.header.frame_id, Time.from_msg(msg.header.stamp), duration
        )
        if pose_pair is None:
            return None, None
        (trans0, yaw0), (trans1, yaw1) = pose_pair

        ranges = np.asarray(msg.ranges, dtype=float)
        indices = np.arange(n_total)
        angles = msg.angle_min + indices * msg.angle_increment

        # 유효하지 않은 측정 제거
        valid = np.isfinite(ranges)
        valid &= ranges > max(msg.range_min, 0.05)
        valid &= ranges < msg.range_max
        ranges = ranges[valid]
        angles = angles[valid]
        indices = indices[valid]
        if len(ranges) == 0:
            return None, None

        # 포인트별 pose 보간 (2D: 위치 선형, yaw 각도 보간) — 전부 벡터화
        if duration > 1e-9:
            f = (indices * time_inc) / duration
        else:
            f = np.zeros(len(indices))
        yaw = yaw0 + f * normalize_angle(yaw1 - yaw0)
        tx = trans0[0] + f * (trans1[0] - trans0[0])
        ty = trans0[1] + f * (trans1[1] - trans0[1])

        x_l = ranges * np.cos(angles)
        y_l = ranges * np.sin(angles)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        x_m = cos_yaw * x_l - sin_yaw * y_l + tx
        y_m = sin_yaw * x_l + cos_yaw * y_l + ty

        return np.column_stack((x_m, y_m)), ranges

    # ------------------------------------------------------------------ pipeline

    def scans_to_obs_point_clouds(self, msg):
        points, ranges = self.transform_scan_to_map(msg)
        if points is None or len(points) < 2:
            return []

        # --- Adaptive Breakpoint Detection (벡터화) ---
        d_phi = msg.angle_increment
        denom = max(math.sin(self.lambda_angle - d_phi), 1e-6)
        consecutive_dist = np.linalg.norm(points[1:] - points[:-1], axis=1)
        d_max = (ranges[1:] * math.sin(d_phi) / denom + 3.0 * self.sigma) / 2.0
        break_indices = np.where(consecutive_dist > d_max)[0] + 1

        clusters = np.split(points, break_indices)
        cluster_ranges = np.split(ranges, break_indices)

        # --- 크기(포인트 수) 필터: 거리 적응형 ---
        kept = []
        for cluster, cluster_r in zip(clusters, cluster_ranges):
            mean_r = float(np.mean(cluster_r))
            required = int(round(self.min_obs_size * self.min_size_ref_dist / max(mean_r, 0.2)))
            required = max(self.min_points_floor, min(self.min_obs_size, required))
            if len(cluster) >= required:
                kept.append(cluster)
        if not kept:
            return []

        # --- 트랙 내부 필터 ---
        # 중간점 하나만 검사하면 코너의 짧은 벽 조각이 통과할 수 있으므로
        # 시작/중간/끝 3점이 모두 트랙 내부일 때만 장애물 후보로 인정
        sample_x = []
        sample_y = []
        for cluster in kept:
            for pt in (cluster[0], cluster[len(cluster) // 2], cluster[-1]):
                sample_x.append(pt[0])
                sample_y.append(pt[1])
        s_points, d_points = self.converter.get_frenet(np.array(sample_x), np.array(sample_y))

        filtered = []
        for idx, cluster in enumerate(kept):
            on_track = all(
                self.laser_point_on_track(
                    float(s_points[3 * idx + k]), float(d_points[3 * idx + k])
                )
                for k in range(3)
            )
            if on_track:
                filtered.append(cluster)

        # --- 인접 클러스터 병합 (부분 가림 등으로 쪼개진 동일 물체) ---
        merged = []
        for cluster in filtered:
            if merged and math.dist(merged[-1][-1], cluster[0]) < self.cluster_merge_dist:
                merged[-1] = np.vstack((merged[-1], cluster))
            else:
                merged.append(cluster)

        return merged

    def publish_breakpoint_markers(self, objects):
        markers = self.clear_marker_array()
        for idx, obj in enumerate(objects):
            color_b = 0.0 if len(objects) == 0 else idx / max(len(objects), 1)
            for marker_id, point in ((idx * 10, obj[0]), (idx * 10 + 2, obj[-1])):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.current_stamp
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.a = 0.5
                marker.color.g = 1.0
                marker.color.b = color_b
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.orientation.w = 1.0
                markers.markers.append(marker)
        self.breakpoints_markers_pub.publish(markers)

    def obs_point_clouds_to_obs_array(self, objects):
        current_obstacles = []
        min_dist = self.min_2_points_dist
        for obstacle_np in objects:
            theta = np.linspace(0.0, np.pi / 2.0 - np.pi / 180.0, 90)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            distance1 = np.dot(obstacle_np, [cos_theta, sin_theta])
            distance2 = np.dot(obstacle_np, [-sin_theta, cos_theta])
            d10 = -distance1 + np.amax(distance1, axis=0)
            d11 = distance1 - np.amin(distance1, axis=0)
            d20 = -distance2 + np.amax(distance2, axis=0)
            d21 = distance2 - np.amin(distance2, axis=0)
            min_array = np.argmin(
                [np.linalg.norm(d10, axis=0), np.linalg.norm(d11, axis=0)], axis=0
            )
            d10 = np.transpose(d10)
            d11 = np.transpose(d11)
            d10[min_array == 1] = d11[min_array == 1]
            d10 = np.transpose(d10)
            min_array = np.argmin(
                [np.linalg.norm(d20, axis=0), np.linalg.norm(d21, axis=0)], axis=0
            )
            d20 = np.transpose(d20)
            d21 = np.transpose(d21)
            d20[min_array == 1] = d21[min_array == 1]
            d20 = np.transpose(d20)
            distances = np.minimum(d10, d20)
            distances[distances < min_dist] = min_dist

            theta_opt = np.argmax(np.sum(np.reciprocal(distances), axis=0)) * np.pi / 180.0
            axis_1 = np.array([np.cos(theta_opt), np.sin(theta_opt)])
            axis_2 = np.array([-np.sin(theta_opt), np.cos(theta_opt)])
            distances1 = np.dot(obstacle_np, axis_1)
            distances2 = np.dot(obstacle_np, axis_2)
            max_dist1 = np.max(distances1)
            min_dist1 = np.min(distances1)
            max_dist2 = np.max(distances2)
            min_dist2 = np.min(distances2)

            center = axis_1 * ((max_dist1 + min_dist1) * 0.5)
            center += axis_2 * ((max_dist2 + min_dist2) * 0.5)
            size = max(max_dist1 - min_dist1, max_dist2 - min_dist2)
            current_obstacles.append(
                DetectedObstacle(
                    float(center[0]), float(center[1]), float(size), float(theta_opt)
                )
            )
        return current_obstacles

    def check_obstacles(self, current_obstacles):
        self.detected_obstacles = []
        for idx, obs in enumerate(current_obstacles):
            if obs.size > self.max_obs_size:
                continue
            obs.id = idx
            self.detected_obstacles.append(obs)

    def publish_obstacles_message(self):
        msg = ObstacleArray()
        msg.header.stamp = self.current_stamp
        msg.header.frame_id = self.map_frame

        if not self.detected_obstacles:
            self.obstacles_msg_pub.publish(msg)
            return

        x_center = [obstacle.center_x for obstacle in self.detected_obstacles]
        y_center = [obstacle.center_y for obstacle in self.detected_obstacles]
        s_points, d_points = self.converter.get_frenet(np.array(x_center), np.array(y_center))

        for idx, obstacle in enumerate(self.detected_obstacles):
            s = float(s_points[idx])
            d = float(d_points[idx])
            obs_msg = ObstacleMessage()
            obs_msg.id = int(obstacle.id)
            obs_msg.s_start = s - obstacle.size / 2.0
            obs_msg.s_end = s + obstacle.size / 2.0
            obs_msg.d_left = d + obstacle.size / 2.0
            obs_msg.d_right = d - obstacle.size / 2.0
            obs_msg.s_center = s
            obs_msg.d_center = d
            obs_msg.size = float(obstacle.size)
            obs_msg.is_actually_a_gap = False
            obs_msg.is_static = True
            obs_msg.is_visible = True
            msg.obstacles.append(obs_msg)
        self.obstacles_msg_pub.publish(msg)

    def publish_obstacles_markers(self):
        markers = self.clear_marker_array()
        for obs in self.detected_obstacles:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.current_stamp
            marker.id = int(obs.id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker_scale = max(float(obs.size), 0.35)
            marker.scale.x = marker_scale
            marker.scale.y = marker_scale
            marker.scale.z = marker_scale
            marker.color.a = 0.5
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.pose.position.x = obs.center_x
            marker.pose.position.y = obs.center_y
            qx, qy, qz, qw = yaw_to_quaternion(obs.theta)
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            markers.markers.append(marker)
        self.obstacles_marker_pub.publish(markers)


def main():
    rclpy.init()
    node = Detect()
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
