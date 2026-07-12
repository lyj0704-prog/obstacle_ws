"""Small numpy-only Frenet helpers shared by detection and tracking."""

import math

import numpy as np


def normalize_s(value, track_length):
    value = value % track_length
    if value > track_length / 2.0:
        value -= track_length
    return value


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class SimpleFrenetConverter:
    def __init__(self, x_points, y_points):
        self.points = np.column_stack((np.asarray(x_points), np.asarray(y_points)))
        if len(self.points) < 2:
            raise ValueError("SimpleFrenetConverter needs at least two waypoints")

        deltas = self.points[1:] - self.points[:-1]
        self.segment_lengths = np.linalg.norm(deltas, axis=1)
        self.segment_lengths[self.segment_lengths < 1e-9] = 1e-9
        self.segment_dirs = deltas / self.segment_lengths[:, None]
        self.s_prefix = np.concatenate(([0.0], np.cumsum(self.segment_lengths)))
        self.track_length = float(self.s_prefix[-1])

    def get_frenet(self, xs, ys):
        scalar = np.isscalar(xs) and np.isscalar(ys)
        query = np.column_stack((np.atleast_1d(xs), np.atleast_1d(ys)))
        starts = self.points[:-1]
        s_values = []
        d_values = []
        for point in query:
            rel = point - starts
            projections = np.sum(rel * self.segment_dirs, axis=1)
            clamped = np.clip(projections, 0.0, self.segment_lengths)
            closest = starts + self.segment_dirs * clamped[:, None]
            idx = int(np.argmin(np.linalg.norm(point - closest, axis=1)))
            tangent = self.segment_dirs[idx]
            normal = np.array([-tangent[1], tangent[0]])
            d_values.append(float(np.dot(point - closest[idx], normal)))
            s_values.append(float((self.s_prefix[idx] + clamped[idx]) % self.track_length))
        if scalar:
            return s_values[0], d_values[0]
        return np.asarray(s_values), np.asarray(d_values)

    def get_cartesian(self, s, d):
        scalar = np.isscalar(s) and np.isscalar(d)
        points = []
        for s_value, d_value in zip(np.atleast_1d(s), np.atleast_1d(d)):
            s_wrapped = float(s_value % self.track_length)
            idx = int(np.searchsorted(self.s_prefix, s_wrapped, side="right") - 1)
            idx = min(max(idx, 0), len(self.segment_lengths) - 1)
            base = self.points[idx] + self.segment_dirs[idx] * (s_wrapped - self.s_prefix[idx])
            normal = np.array([-self.segment_dirs[idx][1], self.segment_dirs[idx][0]])
            points.append(base + normal * float(d_value))
        points = np.asarray(points)
        if scalar:
            return points[0]
        return points[:, 0], points[:, 1]


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
