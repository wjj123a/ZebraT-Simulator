#!/usr/bin/env python3

import math
import time

import rospy
import tf
from costmap_converter.msg import ObstacleArrayMsg
from costmap_converter.msg import ObstacleMsg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix


def _point32(x, y, z=0.0):
    point = Point32()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _clamp_vector(vx, vy, max_speed):
    speed = math.hypot(vx, vy)
    if speed <= max_speed or speed <= 1e-6:
        return vx, vy
    scale = max_speed / speed
    return vx * scale, vy * scale


def cluster_scan_points(scan, cluster_max_gap=0.18, min_points=3, min_range=None, max_range=None):
    lower = scan.range_min if min_range is None else max(float(min_range), float(scan.range_min))
    upper = scan.range_max if max_range is None else min(float(max_range), float(scan.range_max))
    clusters = []
    current = []
    previous = None

    for index, value in enumerate(scan.ranges):
        if not math.isfinite(value) or value < lower or value > upper:
            if len(current) >= min_points:
                clusters.append(current)
            current = []
            previous = None
            continue

        angle = scan.angle_min + index * scan.angle_increment
        point = (float(value) * math.cos(angle), float(value) * math.sin(angle))
        if previous is not None and _distance(point, previous) > cluster_max_gap:
            if len(current) >= min_points:
                clusters.append(current)
            current = []
        current.append(point)
        previous = point

    if len(current) >= min_points:
        clusters.append(current)
    return clusters


class SensorDetection:
    def __init__(self, x, y, radius, point_count):
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)
        self.point_count = int(point_count)


def cluster_to_detection(points, radius_padding=0.08, min_radius=0.12, max_cluster_diameter=1.20):
    if not points:
        return None

    cx = sum(point[0] for point in points) / len(points)
    cy = sum(point[1] for point in points) / len(points)
    radius = max(math.hypot(point[0] - cx, point[1] - cy) for point in points)
    if 2.0 * radius > max_cluster_diameter:
        return None
    return SensorDetection(cx, cy, max(min_radius, radius + radius_padding), len(points))


def build_sensor_obstacle_message(obstacle_id, x, y, radius, vx, vy):
    message = ObstacleMsg()
    message.id = int(obstacle_id)
    message.polygon.points = [_point32(x, y)]
    message.radius = max(0.01, float(radius))
    yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-4 else 0.0
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    message.orientation.x = qx
    message.orientation.y = qy
    message.orientation.z = qz
    message.orientation.w = qw
    message.velocities.twist.linear.x = float(vx)
    message.velocities.twist.linear.y = float(vy)
    return message


class TrackedSensorObstacle:
    def __init__(self, obstacle_id, detection, wall_time, velocity_alpha, max_speed):
        self.obstacle_id = int(obstacle_id)
        self.x = float(detection.x)
        self.y = float(detection.y)
        self.radius = float(detection.radius)
        self.vx = 0.0
        self.vy = 0.0
        self.hits = 1
        self.last_seen_wall_time = float(wall_time)
        self.last_update_wall_time = float(wall_time)
        self.velocity_alpha = min(1.0, max(0.0, float(velocity_alpha)))
        self.max_speed = max(0.05, float(max_speed))
        self.moving_hits = 0
        self.moving_until_wall_time = 0.0

    def update(self, detection, wall_time, motion_speed_threshold=0.12, moving_retain_time=2.0, min_moving_hits=2):
        dt = max(0.0, float(wall_time) - self.last_update_wall_time)
        if dt >= 1e-3:
            measured_vx = (float(detection.x) - self.x) / dt
            measured_vy = (float(detection.y) - self.y) / dt
            measured_vx, measured_vy = _clamp_vector(measured_vx, measured_vy, self.max_speed)
            measured_speed = math.hypot(measured_vx, measured_vy)
            alpha = self.velocity_alpha
            self.vx = alpha * measured_vx + (1.0 - alpha) * self.vx
            self.vy = alpha * measured_vy + (1.0 - alpha) * self.vy
            if measured_speed >= float(motion_speed_threshold):
                self.moving_hits += 1
                if self.moving_hits >= int(min_moving_hits):
                    self.moving_until_wall_time = float(wall_time) + max(0.0, float(moving_retain_time))
            else:
                self.moving_hits = max(0, self.moving_hits - 1)

        self.x = float(detection.x)
        self.y = float(detection.y)
        self.radius = max(float(self.radius), float(detection.radius))
        self.hits += 1
        self.last_seen_wall_time = float(wall_time)
        self.last_update_wall_time = float(wall_time)

    def is_fresh(self, wall_time, timeout):
        return float(wall_time) - self.last_seen_wall_time <= float(timeout)

    def is_moving(self, wall_time):
        return float(wall_time) <= self.moving_until_wall_time

    def distance_to(self, detection):
        return math.hypot(float(detection.x) - self.x, float(detection.y) - self.y)

    def to_message(self, wall_time=None, radius_growth_rate=0.0):
        age = 0.0 if wall_time is None else max(0.0, float(wall_time) - self.last_seen_wall_time)
        return build_sensor_obstacle_message(
            self.obstacle_id,
            self.x + self.vx * age,
            self.y + self.vy * age,
            self.radius + max(0.0, float(radius_growth_rate)) * age,
            self.vx,
            self.vy,
        )


class ScanDynamicObstaclePredictionPublisher:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/move_base/TebLocalPlannerROS/obstacles")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 12.0))
        self.cluster_max_gap = float(rospy.get_param("~cluster_max_gap", 0.22))
        self.min_cluster_points = int(rospy.get_param("~min_cluster_points", 3))
        self.max_cluster_diameter = float(rospy.get_param("~max_cluster_diameter", 1.20))
        self.radius_padding = float(rospy.get_param("~radius_padding", 0.14))
        self.min_radius = float(rospy.get_param("~min_radius", 0.24))
        self.min_range = rospy.get_param("~min_range", None)
        self.max_range = rospy.get_param("~max_range", None)
        self.track_timeout = float(rospy.get_param("~track_timeout", 1.8))
        self.min_track_hits = int(rospy.get_param("~min_track_hits", 1))
        self.association_distance = float(rospy.get_param("~association_distance", 0.65))
        self.velocity_smoothing_alpha = float(rospy.get_param("~velocity_smoothing_alpha", 0.45))
        self.max_obstacle_speed = float(rospy.get_param("~max_obstacle_speed", 2.0))
        self.stale_radius_growth_rate = float(rospy.get_param("~stale_radius_growth_rate", 0.12))
        self.publish_stationary_obstacles = bool(rospy.get_param("~publish_stationary_obstacles", False))
        self.min_publish_speed = float(rospy.get_param("~min_publish_speed", 0.12))
        self.min_moving_hits = int(rospy.get_param("~min_moving_hits", 2))
        self.moving_track_retain_time = float(rospy.get_param("~moving_track_retain_time", 2.0))

        self._tf = tf.TransformListener()
        self._publisher = rospy.Publisher(self.output_topic, ObstacleArrayMsg, queue_size=1)
        self._tracks = {}
        self._next_id = 1
        self._last_scan_stamp = rospy.Time(0)

        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.publish_rate)), self._publish_timer)
        rospy.loginfo(
            "Publishing sensor-derived dynamic obstacle predictions from %s to %s",
            self.scan_topic,
            self.output_topic,
        )

    def _transform_detection(self, detection, source_frame):
        if not source_frame or source_frame == self.output_frame:
            return detection

        try:
            translation, rotation = self._tf.lookupTransform(self.output_frame, source_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Could not transform scan obstacle from %s to %s: %s",
                source_frame,
                self.output_frame,
                exc,
            )
            return None

        matrix = quaternion_matrix(rotation)
        x = matrix[0][0] * detection.x + matrix[0][1] * detection.y + float(translation[0])
        y = matrix[1][0] * detection.x + matrix[1][1] * detection.y + float(translation[1])
        return SensorDetection(x, y, detection.radius, detection.point_count)

    def _detections_from_scan(self, scan):
        clusters = cluster_scan_points(
            scan,
            cluster_max_gap=self.cluster_max_gap,
            min_points=self.min_cluster_points,
            min_range=self.min_range,
            max_range=self.max_range,
        )
        detections = []
        for cluster in clusters:
            detection = cluster_to_detection(
                cluster,
                radius_padding=self.radius_padding,
                min_radius=self.min_radius,
                max_cluster_diameter=self.max_cluster_diameter,
            )
            if detection is None:
                continue
            transformed = self._transform_detection(detection, scan.header.frame_id)
            if transformed is not None:
                detections.append(transformed)
        return detections

    def _new_track(self, detection, wall_time):
        obstacle_id = self._next_id
        self._next_id += 1
        self._tracks[obstacle_id] = TrackedSensorObstacle(
            obstacle_id,
            detection,
            wall_time,
            self.velocity_smoothing_alpha,
            self.max_obstacle_speed,
        )

    def _update_tracks(self, detections, wall_time):
        unused_track_ids = set(self._tracks.keys())
        for detection in detections:
            best_id = None
            best_distance = None
            for obstacle_id in list(unused_track_ids):
                track = self._tracks[obstacle_id]
                if not track.is_fresh(wall_time, self.track_timeout):
                    continue
                distance = track.distance_to(detection)
                gate = self.association_distance + max(track.radius, detection.radius)
                if distance <= gate and (best_distance is None or distance < best_distance):
                    best_id = obstacle_id
                    best_distance = distance

            if best_id is None:
                self._new_track(detection, wall_time)
                continue

            self._tracks[best_id].update(
                detection,
                wall_time,
                self.min_publish_speed,
                self.moving_track_retain_time,
                self.min_moving_hits,
            )
            unused_track_ids.remove(best_id)

        stale = [
            obstacle_id
            for obstacle_id, track in self._tracks.items()
            if not track.is_fresh(wall_time, self.track_timeout)
        ]
        for obstacle_id in stale:
            del self._tracks[obstacle_id]

    def _scan_callback(self, scan):
        self._last_scan_stamp = scan.header.stamp
        detections = self._detections_from_scan(scan)
        self._update_tracks(detections, time.monotonic())

    def _publish_timer(self, _event):
        now = time.monotonic()
        message = ObstacleArrayMsg()
        message.header.stamp = self._last_scan_stamp if self._last_scan_stamp != rospy.Time(0) else rospy.Time.now()
        message.header.frame_id = self.output_frame
        for track in self._tracks.values():
            if not track.is_fresh(now, self.track_timeout) or track.hits < self.min_track_hits:
                continue
            if not self.publish_stationary_obstacles and not track.is_moving(now):
                continue
            if track.is_fresh(now, self.track_timeout):
                message.obstacles.append(track.to_message(now, self.stale_radius_growth_rate))
        self._publisher.publish(message)


def main():
    rospy.init_node("scan_dynamic_obstacle_predictions")
    ScanDynamicObstaclePredictionPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
