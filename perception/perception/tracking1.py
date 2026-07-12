#!/usr/bin/env python3
"""
정적/동적 장애물 트래킹 노드 (Frenet 좌표계 칼만필터 기반).

Hokuyo UST-10LX (40Hz) 대응 수정 사항:

1. [Hz] 타이머 폴링 -> 검출 메시지 콜백 구동으로 변경. detect가 40Hz로 publish하면
   트래킹도 40Hz로 갱신·publish. dt는 측정 stamp 차이로 계산하므로 detect.rate를
   바꿔도 별도 수정 불필요.
2. [Hz] TTL을 "프레임 수" -> "초 단위"로 변경 (ttl_static_sec / ttl_dynamic_sec).
   프레임 수 기반이었다면 40Hz에서 ttl_static=3은 75ms가 되어 정적 장애물이
   과도하게 빨리 삭제됨. 이제 처리율과 무관하게 동일하게 동작.
3. [Hz] 분류용 측정 히스토리를 "최근 30개" -> "시간 윈도우"(classification_window,
   기본 3.0s)로 변경. 40Hz에서 30개는 0.75s에 불과해 저속 동적 장애물이 정적으로
   오분류될 수 있었음.
4. 분류 안정 장치: 히스토리 시간 폭이 min_obs_time(0.5s) 이상일 때만 std 기반
   분류 수행. 대신 KF 속도 |vs|가 vs_dynamic_threshold(0.6m/s)를 넘으면 즉시
   동적으로 분류 (고속 상대 차는 0.1~0.2s 내 동적 판정).
5. RViz 마커는 viz_rate(10Hz) 타이머로만 publish (RViz 부하 절감).
   같은 타이머가 워치독 역할: 측정이 0.15s 이상 끊기면 coasting 예측 후 publish.

(이전 버전에서 이미 반영: KF [s,vs,d,vd], 예측 위치 기반 전역 최근접 연관,
 s 랩어라운드, 가시성 기반 TTL, 지연 보상, min_hits 고스트 억제,
 미분류=동적 취급, /perception/obstacles에 정적+동적 포함)
"""

import math
import time

import numpy as np
import rclpy
from f110_msgs.msg import Obstacle, ObstacleArray, WpntArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from .frenet_utils import SimpleFrenetConverter, normalize_angle, normalize_s, quaternion_to_yaw


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class KalmanTrack:
    """Frenet 좌표계 등속(CV) 모델 칼만필터 트랙. 상태: [s, vs, d, vd]."""

    H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])

    def __init__(self, track_id, measurement, t_sec, lap, sigma_meas, init_vel_std):
        s = float(measurement.s_center)
        d = float(measurement.d_center)
        self.id = track_id
        self.x = np.array([s, 0.0, d, 0.0])
        self.P = np.diag([sigma_meas**2, init_vel_std**2, sigma_meas**2, init_vel_std**2])
        self.history = [(t_sec, s, d)]  # (시각, s, d) — 분류용
        self.mean = [s, d]  # 정적 장애물 publish용 (원형 평균)
        self.size = float(measurement.size)
        self.time_since_match = 0.0  # "보이는데 미매칭" 누적 시간 (초)
        self.time_unseen = 0.0  # 가시성 무관 미매칭 누적 시간 (초) — 유령 트랙 만료용
        self.vs_exceed = 0  # |vs|가 동적 문턱을 연속으로 넘은 횟수 (디바운스)
        self.hits = 1
        self.current_lap = lap
        self.is_visible = True
        self.is_in_front = True
        self.static_flag = None  # None=미분류, True=정적, False=동적

    # -------------------------------------------------- Kalman filter

    def predict(self, dt, sigma_accel):
        F = np.array(
            [
                [1.0, dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, dt],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        q11 = 0.25 * dt**4 * sigma_accel**2
        q12 = 0.50 * dt**3 * sigma_accel**2
        q22 = dt**2 * sigma_accel**2
        Q = np.array(
            [
                [q11, q12, 0.0, 0.0],
                [q12, q22, 0.0, 0.0],
                [0.0, 0.0, q11, q12],
                [0.0, 0.0, q12, q22],
            ]
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(
        self,
        measurement,
        t_sec,
        track_length,
        sigma_meas,
        window,
        min_nb_meas,
        min_obs_time,
        min_std,
        max_std,
        vs_dynamic_threshold,
        vs_dynamic_min_count,
    ):
        s_meas = float(measurement.s_center)
        d_meas = float(measurement.d_center)

        # 트랙별 랩 카운트: raw s가 결승선을 넘어 크게 감소하면 랩 증가
        if s_meas - self.history[-1][1] < -track_length / 2.0:
            self.current_lap += 1

        # s 측정을 예측 상태 근방으로 언랩 (결승선 랩어라운드 처리)
        z = np.array([self.x[0] + normalize_s(s_meas - self.x[0], track_length), d_meas])
        R = np.eye(2) * sigma_meas**2
        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.x[0] %= track_length

        # 시간 윈도우 기반 히스토리 (처리율 무관)
        self.history.append((t_sec, s_meas, d_meas))
        cutoff = t_sec - window
        while len(self.history) > 2 and self.history[0][0] < cutoff:
            self.history.pop(0)

        self.size = float(measurement.size)
        self.time_since_match = 0.0
        self.time_unseen = 0.0
        self.hits += 1
        self.is_visible = True
        self.update_mean(track_length)
        self.classify(
            track_length, min_nb_meas, min_obs_time, min_std, max_std,
            vs_dynamic_threshold, vs_dynamic_min_count,
        )

    # -------------------------------------------------- statistics / classification

    def update_mean(self, track_length):
        d_values = np.asarray([h[2] for h in self.history])
        self.mean[1] = float(np.mean(d_values))

        angles = np.asarray([h[1] for h in self.history]) * 2.0 * math.pi / track_length
        mean_angle = math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))
        mean_s = mean_angle * track_length / (2.0 * math.pi)
        self.mean[0] = mean_s if mean_s >= 0.0 else mean_s + track_length

    def classify(
        self, track_length, min_nb_meas, min_obs_time, min_std, max_std,
        vs_dynamic_threshold, vs_dynamic_min_count,
    ):
        if len(self.history) <= min_nb_meas:
            return

        # 속도 기반 조기 동적 판정 (디바운스): |vs|가 문턱을 "연속으로" 넘어야
        # 동적으로 분류. 주행 중 시점 변화로 클러스터 중심이 한두 프레임 튀는
        # 것에는 반응하지 않고, 진짜 움직이는 상대 차(수십 프레임 연속 초과)만 잡음
        if abs(self.x[1]) > vs_dynamic_threshold:
            self.vs_exceed += 1
            if self.vs_exceed >= vs_dynamic_min_count:
                self.static_flag = False
                return
        else:
            self.vs_exceed = 0

        # std 기반 분류는 관측 시간 폭이 충분할 때만 (40Hz에서 수 프레임 만에
        # 오분류되는 것을 방지)
        time_span = self.history[-1][0] - self.history[0][0]
        if time_span < min_obs_time:
            return

        s_std = math.sqrt(
            np.mean([normalize_s(h[1] - self.mean[0], track_length) ** 2 for h in self.history])
        )
        d_std = float(np.std([h[2] for h in self.history]))
        if s_std < min_std and d_std < min_std:
            self.static_flag = True
        elif s_std > max_std or d_std > max_std:
            self.static_flag = False

    def gate_distance(self, measurement, track_length):
        """예측 위치와 측정치 사이 거리 (s 랩어라운드 처리)."""
        ds = normalize_s(float(measurement.s_center) - self.x[0], track_length)
        dd = float(measurement.d_center) - self.x[2]
        return math.hypot(ds, dd)

    def treated_as_dynamic(self, unclassified_as_static):
        if self.static_flag is False:
            return True
        if self.static_flag is None and not unclassified_as_static:
            return True
        return False


class StaticDynamic(Node):
    def __init__(self):
        super().__init__("tracking")

        self.declare_parameter("measure", False)
        self.declare_parameter("tracking.rate", 40.0)  # 예상 측정 주기 (dt 폴백용)
        self.declare_parameter("tracking.viz_rate", 10.0)
        self.declare_parameter("tracking.max_dist", 1.0)
        self.declare_parameter("tracking.ttl_static_sec", 0.3)
        self.declare_parameter("tracking.ttl_dynamic_sec", 4.0)
        self.declare_parameter("tracking.min_nb_meas", 5)
        self.declare_parameter("tracking.min_hits", 5)
        self.declare_parameter("tracking.min_obs_time", 0.5)
        self.declare_parameter("tracking.classification_window", 3.0)
        self.declare_parameter("tracking.min_std", 0.16)
        self.declare_parameter("tracking.max_std", 0.2)
        self.declare_parameter("tracking.vs_dynamic_threshold", 0.6)
        self.declare_parameter("tracking.vs_dynamic_min_count", 4)
        self.declare_parameter("tracking.track_merge_dist", 0.5)
        self.declare_parameter("tracking.ttl_hidden_sec", 60.0)
        self.declare_parameter("tracking.dist_infront", 8.0)
        self.declare_parameter("tracking.publish_static", True)
        self.declare_parameter("tracking.sigma_meas", 0.06)
        self.declare_parameter("tracking.sigma_accel", 3.0)
        self.declare_parameter("tracking.init_vel_std", 2.0)
        self.declare_parameter("tracking.unclassified_as_static", False)
        self.declare_parameter("tracking.visibility_margin", 0.4)
        self.declare_parameter("tracking.max_viewing_distance", 9.0)  # detect와 동일하게 유지
        self.declare_parameter("map_frame", "map")

        self.measuring = bool(self.get_parameter("measure").value)
        self.rate = float(self.get_parameter("tracking.rate").value)
        self.viz_rate = float(self.get_parameter("tracking.viz_rate").value)
        self.max_dist = float(self.get_parameter("tracking.max_dist").value)
        self.ttl_static_sec = float(self.get_parameter("tracking.ttl_static_sec").value)
        self.ttl_dynamic_sec = float(self.get_parameter("tracking.ttl_dynamic_sec").value)
        self.min_nb_meas = int(self.get_parameter("tracking.min_nb_meas").value)
        self.min_hits = int(self.get_parameter("tracking.min_hits").value)
        self.min_obs_time = float(self.get_parameter("tracking.min_obs_time").value)
        self.classification_window = float(
            self.get_parameter("tracking.classification_window").value
        )
        self.min_std = float(self.get_parameter("tracking.min_std").value)
        self.max_std = float(self.get_parameter("tracking.max_std").value)
        self.vs_dynamic_threshold = float(
            self.get_parameter("tracking.vs_dynamic_threshold").value
        )
        self.vs_dynamic_min_count = int(self.get_parameter("tracking.vs_dynamic_min_count").value)
        self.track_merge_dist = float(self.get_parameter("tracking.track_merge_dist").value)
        self.ttl_hidden_sec = float(self.get_parameter("tracking.ttl_hidden_sec").value)
        self.dist_infront = float(self.get_parameter("tracking.dist_infront").value)
        self.publish_static = bool(self.get_parameter("tracking.publish_static").value)
        self.sigma_meas = float(self.get_parameter("tracking.sigma_meas").value)
        self.sigma_accel = float(self.get_parameter("tracking.sigma_accel").value)
        self.init_vel_std = float(self.get_parameter("tracking.init_vel_std").value)
        self.unclassified_as_static = bool(
            self.get_parameter("tracking.unclassified_as_static").value
        )
        self.visibility_margin = float(self.get_parameter("tracking.visibility_margin").value)
        self.max_viewing_distance = float(
            self.get_parameter("tracking.max_viewing_distance").value
        )
        self.map_frame = self.get_parameter("map_frame").value

        self.tracked_obstacles = []
        self.waypoints = None
        self.track_length = None
        self.converter = None
        self.car_s = 0.0
        self.last_car_s = 0.0
        self.current_lap = 0
        self.car_position = np.array([0.0, 0.0])
        self.car_yaw = 0.0
        self.current_stamp = self.get_clock().now().to_msg()
        self.last_meas_time = None  # 마지막으로 처리한 측정 stamp (sec)
        self.scan = None
        self.current_id = 1

        # 검출 메시지 콜백 구동 (타이머 폴링 제거)
        self.create_subscription(
            ObstacleArray,
            "/perception/detection/raw_obstacles",
            self.obstacle_cb,
            10,
        )
        self.create_subscription(WpntArray, "/global_waypoints", self.path_cb, 10)
        self.create_subscription(
            Odometry, "/car_state/frenet/odom", self.car_state_cb, 10
        )
        self.create_subscription(Odometry, "/car_state/odom", self.car_state_glob_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scans_cb, 10)

        self.static_dynamic_marker_pub = self.create_publisher(
            MarkerArray, "/perception/static_dynamic_marker_pub", 10
        )
        self.estimated_obstacles_pub = self.create_publisher(
            ObstacleArray, "/perception/obstacles", 10
        )
        self.raw_opponent_pub = self.create_publisher(
            ObstacleArray, "/perception/raw_obstacles", 10
        )
        if self.measuring:
            self.latency_pub = self.create_publisher(Float32, "/perception/tracking/latency", 10)

        # 마커 publish + 측정 두절 시 coasting 워치독
        self.viz_timer = self.create_timer(1.0 / self.viz_rate, self.on_viz_timer)
        self.get_logger().info(
            "[Opponent Tracking]: ROS2 node ready (event-driven Kalman tracker)"
        )

    # ------------------------------------------------------------------ callbacks

    def obstacle_cb(self, data):
        if self.converter is None or self.track_length is None:
            return

        if self.measuring:
            start = time.perf_counter()

        self.current_stamp = data.header.stamp
        t_sec = stamp_to_sec(data.header.stamp)

        # dt: 측정 stamp 차이 (비정상이면 1/rate 폴백)
        nominal = 1.0 / self.rate
        if self.last_meas_time is None:
            dt = nominal
        else:
            dt = t_sec - self.last_meas_time
            if not (0.001 <= dt <= 0.5):
                dt = nominal
        self.last_meas_time = t_sec

        self.update_tracks(list(data.obstacles), t_sec, dt)
        self.publish_obstacles(self.compute_publish_dt())

        if self.measuring:
            latency = Float32()
            latency.data = float(time.perf_counter() - start)
            self.latency_pub.publish(latency)

    def path_cb(self, data):
        if not data.wpnts:
            return
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        if self.converter is None:
            self.converter = SimpleFrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
            self.track_length = self.converter.track_length
            self.get_logger().info("[Tracking]: initialized SimpleFrenetConverter")

    def car_state_cb(self, data):
        self.car_s = float(data.pose.pose.position.x)

    def car_state_glob_cb(self, data):
        self.car_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.car_yaw = quaternion_to_yaw(data.pose.pose.orientation)

    def scans_cb(self, data):
        self.scan = data

    # ------------------------------------------------------------------ helpers

    def lap_update(self):
        if self.track_length and self.car_s - self.last_car_s < -self.track_length / 2.0:
            self.current_lap += 1
        self.last_car_s = self.car_s

    def compute_publish_dt(self):
        """지연 보상용: 측정 시각 -> 현재 시각."""
        if self.last_meas_time is None:
            return 0.0
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return min(max(now_sec - self.last_meas_time, 0.0), 0.3)

    def expected_visible(self, s_track, x, y):
        """
        트랙 위치가 "검출 노드가 측정을 줄 수 있는" 위치인지 판단.

        - detect는 s 기준 전방 max_viewing_distance 밖의 장애물을 publish하지
          않으므로, 그 영역 밖이면 (직선거리로 보이더라도) False (-> TTL 유지).
          이 조건이 없으면 코너 건너편의 장애물이 "보이는데 측정이 없다"로
          오판되어 랩마다 트랙이 죽고 새 ID가 생기는 churn이 발생함.
        - 시야각(270°) 밖 또는 벽/다른 물체에 가려진 경우 False (-> TTL 유지)
        - 그 위치까지 스캔이 뚫려 있는데 검출이 없으면 True (-> TTL 감소)
        """
        if normalize_s(s_track - self.car_s, self.track_length) > self.max_viewing_distance:
            return False
        if self.scan is None:
            return True
        dx = float(x) - float(self.car_position[0])
        dy = float(y) - float(self.car_position[1])
        r = math.hypot(dx, dy)
        if r < 0.1:
            return True
        bearing = normalize_angle(math.atan2(dy, dx) - self.car_yaw)
        if bearing < self.scan.angle_min + 0.05 or bearing > self.scan.angle_max - 0.05:
            return False
        idx = int((bearing - self.scan.angle_min) / self.scan.angle_increment)
        lo = max(0, idx - 2)
        hi = min(len(self.scan.ranges), idx + 3)
        window = [float(rr) for rr in self.scan.ranges[lo:hi] if math.isfinite(rr)]
        if not window:
            return False
        return min(window) > r - self.visibility_margin

    # ------------------------------------------------------------------ tracking core

    def associate(self, measurements):
        """전역 최근접(global nearest neighbor) 연관."""
        pairs = []
        for t_idx, tracked in enumerate(self.tracked_obstacles):
            for m_idx, measurement in enumerate(measurements):
                dist = tracked.gate_distance(measurement, self.track_length)
                if dist < self.max_dist:
                    pairs.append((dist, t_idx, m_idx))
        pairs.sort(key=lambda item: item[0])

        assignment = {}
        used_tracks = set()
        used_meas = set()
        for dist, t_idx, m_idx in pairs:
            if t_idx in used_tracks or m_idx in used_meas:
                continue
            assignment[t_idx] = m_idx
            used_tracks.add(t_idx)
            used_meas.add(m_idx)

        unmatched = [m for idx, m in enumerate(measurements) if idx not in used_meas]
        return assignment, unmatched

    def update_tracks(self, measurements, t_sec, dt):
        self.lap_update()

        # 1) 모든 트랙 예측 (coasting 포함)
        for tracked in self.tracked_obstacles:
            tracked.predict(dt, self.sigma_accel)

        # 2) 예측 위치 기준 연관
        assignment, unmatched_meas = self.associate(measurements)

        # 3) 매칭된 트랙 갱신 / 미매칭 트랙 TTL 처리 (초 단위)
        removals = []
        for t_idx, tracked in enumerate(self.tracked_obstacles):
            if t_idx in assignment:
                tracked.update(
                    measurements[assignment[t_idx]],
                    t_sec,
                    self.track_length,
                    self.sigma_meas,
                    self.classification_window,
                    self.min_nb_meas,
                    self.min_obs_time,
                    self.min_std,
                    self.max_std,
                    self.vs_dynamic_threshold,
                    self.vs_dynamic_min_count,
                )
            else:
                tracked.is_visible = False
                tracked.time_unseen += dt
                if tracked.static_flag is False:
                    # 동적: 가려져 있어도 coasting은 유한해야 하므로 항상 누적
                    tracked.time_since_match += dt
                    if tracked.time_since_match > self.ttl_dynamic_sec:
                        removals.append(tracked)
                else:
                    # 정적/미분류: "보여야 하는데 안 보일 때"만 누적 (가림 시 유지)
                    x, y = self.converter.get_cartesian(tracked.x[0], tracked.x[2])
                    if self.expected_visible(tracked.x[0], x, y):
                        tracked.time_since_match += dt
                    if (
                        tracked.time_since_match > self.ttl_static_sec
                        or tracked.time_unseen > self.ttl_hidden_sec
                    ):
                        # ttl_hidden_sec: 시야 밖이라도 이 시간 넘게 재관측이 없으면
                        # 만료 (랩마다 쌓이는 유령 트랙 방지, 랩타임보다 길게 설정)
                        removals.append(tracked)

            dist_in_front = normalize_s(tracked.x[0] - self.car_s, self.track_length)
            tracked.is_in_front = 0.0 < dist_in_front < self.dist_infront

        for tracked in removals:
            self.tracked_obstacles.remove(tracked)

        # 4) 중복 트랙 병합: 두 트랙의 추정 위치가 track_merge_dist 이내로 겹치면
        #    같은 장애물이므로 하나로 흡수. (랩 사이 로컬라이제이션 드리프트로
        #    재접근 시 새 트랙이 생기고 옛 트랙이 시야 밖에서 살아남는 것을 정리)
        #    관측 이력(hits)이 많은 쪽을 남기고, ID 연속성을 위해 더 작은 ID를 승계.
        self.merge_duplicate_tracks()

        # 5) 미매칭 측정 -> 새 트랙
        for measurement in unmatched_meas:
            self.tracked_obstacles.append(
                KalmanTrack(
                    self.current_id,
                    measurement,
                    t_sec,
                    self.current_lap,
                    self.sigma_meas,
                    self.init_vel_std,
                )
            )
            self.current_id += 1

    def merge_duplicate_tracks(self):
        removals = set()
        n = len(self.tracked_obstacles)
        for i in range(n):
            for j in range(i + 1, n):
                a = self.tracked_obstacles[i]
                b = self.tracked_obstacles[j]
                if id(a) in removals or id(b) in removals:
                    continue
                ds = normalize_s(a.x[0] - b.x[0], self.track_length)
                dd = a.x[2] - b.x[2]
                if math.hypot(ds, dd) >= self.track_merge_dist:
                    continue
                # 관측 이력이 많은(오래 확인된) 트랙을 남김
                survivor, absorbed = (a, b) if a.hits >= b.hits else (b, a)
                survivor.id = min(a.id, b.id)  # ID 연속성 유지
                survivor.hits += absorbed.hits
                # 최근 정보를 가진 쪽의 상태를 우선 (미매칭 시간이 짧은 쪽)
                if absorbed.time_unseen < survivor.time_unseen:
                    survivor.x = absorbed.x.copy()
                    survivor.P = absorbed.P.copy()
                    survivor.time_since_match = absorbed.time_since_match
                    survivor.time_unseen = absorbed.time_unseen
                    survivor.is_visible = absorbed.is_visible
                removals.add(id(absorbed))
        if removals:
            self.tracked_obstacles = [t for t in self.tracked_obstacles if id(t) not in removals]

    # ------------------------------------------------------------------ publishing

    def publish_position(self, tracked, publish_dt):
        """
        Publish용 (s, d, vs, vd)를 계산한다.

        동적은 필터 상태를 현재 시각으로 예측하고 정적은 원형 평균을 사용한다.
        """
        if tracked.treated_as_dynamic(self.unclassified_as_static):
            s = (tracked.x[0] + tracked.x[1] * publish_dt) % self.track_length
            d = tracked.x[2] + tracked.x[3] * publish_dt
            return s, d, float(tracked.x[1]), float(tracked.x[3]), False
        return tracked.mean[0] % self.track_length, tracked.mean[1], 0.0, 0.0, True

    def make_obstacle_msg(self, tracked, publish_dt):
        s, d, vs, vd, is_static = self.publish_position(tracked, publish_dt)
        obs_msg = Obstacle()
        obs_msg.id = int(tracked.id)
        obs_msg.size = float(tracked.size)
        obs_msg.vs = vs
        obs_msg.vd = vd
        obs_msg.is_static = is_static
        obs_msg.is_visible = bool(tracked.is_visible)
        obs_msg.is_actually_a_gap = False
        obs_msg.s_center = float(s)
        obs_msg.d_center = float(d)
        obs_msg.s_start = obs_msg.s_center - obs_msg.size / 2.0
        obs_msg.s_end = obs_msg.s_center + obs_msg.size / 2.0
        obs_msg.d_right = obs_msg.d_center - obs_msg.size / 2.0
        obs_msg.d_left = obs_msg.d_center + obs_msg.size / 2.0
        return obs_msg

    def publish_obstacles(self, publish_dt):
        obstacle_array = ObstacleArray()
        obstacle_array.header.frame_id = self.map_frame
        obstacle_array.header.stamp = self.current_stamp

        raw_array = ObstacleArray()
        raw_array.header = obstacle_array.header

        for tracked in self.tracked_obstacles:
            if tracked.hits < self.min_hits:
                continue  # 미확인 트랙(고스트) publish 억제
            obs_msg = self.make_obstacle_msg(tracked, publish_dt)
            if not obs_msg.is_static:
                obstacle_array.obstacles.append(obs_msg)
                raw_array.obstacles.append(obs_msg)
            elif self.publish_static:
                obstacle_array.obstacles.append(obs_msg)

        self.estimated_obstacles_pub.publish(obstacle_array)
        self.raw_opponent_pub.publish(raw_array)

    def publish_markers(self, publish_dt):
        markers_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers_array.markers.append(clear)

        for tracked in self.tracked_obstacles:
            if tracked.static_flag is True and not self.publish_static:
                continue

            s, d, _, _, _ = self.publish_position(tracked, publish_dt)
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.current_stamp
            marker.id = int(tracked.id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            scale = 0.8 if tracked.is_in_front else 0.55
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.color.a = 0.6

            if tracked.static_flag is False:
                marker.color.r = 1.0  # 동적: 빨강
            elif tracked.static_flag is True:
                marker.color.g = 1.0  # 정적: 초록
            else:
                marker.color.r = 1.0  # 미분류: 핑크색
                marker.color.b = 1.0

            x, y = self.converter.get_cartesian(s, d)
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.orientation.w = 1.0
            markers_array.markers.append(marker)
        self.static_dynamic_marker_pub.publish(markers_array)

    # ------------------------------------------------------------------ viz / watchdog

    def on_viz_timer(self):
        if self.converter is None or self.track_length is None:
            return

        # 워치독: 측정이 끊기면 coasting 예측 후 publish 유지
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_meas_time is not None and (now_sec - self.last_meas_time) > 0.15:
            dt = now_sec - self.last_meas_time
            dt = min(dt, 0.5)
            for tracked in self.tracked_obstacles:
                tracked.predict(dt, self.sigma_accel)
                if tracked.static_flag is False:
                    tracked.time_since_match += dt
            self.tracked_obstacles = [
                t
                for t in self.tracked_obstacles
                if not (t.static_flag is False and t.time_since_match > self.ttl_dynamic_sec)
            ]
            self.last_meas_time = now_sec
            self.publish_obstacles(0.0)

        self.publish_markers(self.compute_publish_dt())


def main():
    rclpy.init()
    node = StaticDynamic()
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
