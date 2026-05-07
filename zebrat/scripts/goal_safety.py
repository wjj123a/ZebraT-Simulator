#!/usr/bin/env python3

import copy
import math
import time

import rospy
import tf
from costmap_converter.msg import ObstacleArrayMsg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion


def normalize_topic_list(value):
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item) for item in value]


class ResolvedGoal:
    def __init__(self, pose, adjusted=False, waited=False, reason="", blocked=False):
        self.pose = pose
        self.adjusted = adjusted
        self.waited = waited
        self.reason = reason
        self.blocked = blocked


class CostmapGoalResolver:
    def __init__(
        self,
        costmap_topics=None,
        occupied_threshold=65,
        unknown_is_occupied=True,
        target_check_radius=0.35,
        search_radius=1.2,
        search_step=0.10,
        wait_timeout=6.0,
        wait_check_period=0.5,
        costmap_wait_timeout=10.0,
        use_dynamic_routes=False,
        dynamic_route_inflation=0.42,
        use_dynamic_obstacle_predictions=True,
        dynamic_obstacle_topics=None,
        dynamic_obstacle_path_inflation=0.42,
        dynamic_obstacle_prediction_timeout=1.0,
        dynamic_obstacle_prediction_horizon=1.5,
        allow_unknown_fallback=False,
        unknown_fallback_use_dynamic_obstacle_predictions=False,
        base_frame="base_footprint",
    ):
        self.costmap_topics = costmap_topics or [
            "/move_base/local_costmap/costmap",
            "/move_base/global_costmap/costmap",
        ]
        self.occupied_threshold = int(occupied_threshold)
        self.unknown_is_occupied = bool(unknown_is_occupied)
        self.target_check_radius = float(target_check_radius)
        self.search_radius = float(search_radius)
        self.search_step = float(search_step)
        self.wait_timeout = float(wait_timeout)
        self.wait_check_period = float(wait_check_period)
        self.costmap_wait_timeout = float(costmap_wait_timeout)
        self.use_dynamic_routes = bool(use_dynamic_routes)
        self.dynamic_route_inflation = float(dynamic_route_inflation)
        self.use_dynamic_obstacle_predictions = bool(use_dynamic_obstacle_predictions)
        self.dynamic_obstacle_topics = normalize_topic_list(
            dynamic_obstacle_topics or ["/move_base/TebLocalPlannerROS/obstacles"]
        )
        self.dynamic_obstacle_path_inflation = float(dynamic_obstacle_path_inflation)
        self.dynamic_obstacle_prediction_timeout = float(dynamic_obstacle_prediction_timeout)
        self.dynamic_obstacle_prediction_horizon = max(0.0, float(dynamic_obstacle_prediction_horizon))
        self.allow_unknown_fallback = bool(allow_unknown_fallback)
        self.unknown_fallback_use_dynamic_obstacle_predictions = bool(
            unknown_fallback_use_dynamic_obstacle_predictions
        )
        self.base_frame = str(base_frame)

        self._costmaps = {}
        self._dynamic_obstacles = {}
        self._tf = tf.TransformListener()
        self._subscribers = [
            rospy.Subscriber(topic, OccupancyGrid, self._costmap_callback, callback_args=topic, queue_size=1)
            for topic in self.costmap_topics
        ]
        if self.use_dynamic_obstacle_predictions:
            self._subscribers.extend(
                rospy.Subscriber(topic, ObstacleArrayMsg, self._dynamic_obstacle_callback, callback_args=topic, queue_size=1)
                for topic in self.dynamic_obstacle_topics
            )
        self._dynamic_route_segments = self._load_dynamic_route_segments() if self.use_dynamic_routes else []

    def _costmap_callback(self, message, topic):
        self._costmaps[topic] = message

    def _dynamic_obstacle_callback(self, message, topic):
        self._dynamic_obstacles[topic] = (message, time.monotonic())

    def wait_for_costmaps(self):
        deadline = time.monotonic() + self.costmap_wait_timeout
        required_count = len(self.costmap_topics)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if len(self._costmaps) >= required_count:
                return True
            time.sleep(0.1)
        return len(self._costmaps) > 0

    @staticmethod
    def _pose_in_frame(pose, frame_id):
        if not pose.header.frame_id or pose.header.frame_id == frame_id:
            return pose
        return None

    def _transform_pose(self, pose, frame_id):
        if not pose.header.frame_id or pose.header.frame_id == frame_id:
            transformed = PoseStamped()
            transformed.header.frame_id = frame_id
            transformed.header.stamp = rospy.Time(0)
            transformed.pose = pose.pose
            return transformed

        latest_pose = PoseStamped()
        latest_pose.header.frame_id = pose.header.frame_id
        latest_pose.header.stamp = rospy.Time(0)
        latest_pose.pose = pose.pose
        try:
            self._tf.waitForTransform(frame_id, latest_pose.header.frame_id, rospy.Time(0), rospy.Duration(0.2))
            return self._tf.transformPose(frame_id, latest_pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Could not transform goal from %s to %s: %s",
                latest_pose.header.frame_id,
                frame_id,
                exc,
            )
            return None

    @staticmethod
    def _world_to_cell(grid, x, y):
        origin = grid.info.origin
        orientation = origin.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        dx = x - origin.position.x
        dy = y - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if grid.info.resolution <= 0.0:
            return None
        mx = int(math.floor(local_x / grid.info.resolution))
        my = int(math.floor(local_y / grid.info.resolution))
        if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
            return None
        return mx, my

    def _cell_is_blocked(self, value, unknown_is_occupied=None):
        if value < 0:
            if unknown_is_occupied is None:
                unknown_is_occupied = self.unknown_is_occupied
            return bool(unknown_is_occupied)
        return value >= self.occupied_threshold

    @staticmethod
    def _waypoint_xy(item):
        if isinstance(item, dict):
            return float(item["x"]), float(item["y"])
        return float(item[0]), float(item[1])

    @staticmethod
    def _obstacle_radius(spec):
        if "radius" in spec:
            return float(spec["radius"])
        size = spec.get("size", [0.7, 0.4, 0.5])
        return math.hypot(float(size[0]) * 0.5, float(size[1]) * 0.5)

    def _load_dynamic_route_segments(self):
        try:
            profile_name = rospy.get_param("/dynamic_obstacles/profile", None)
            profiles = rospy.get_param("/dynamic_obstacles/profiles", {})
        except Exception:
            return []
        if not profile_name or profile_name not in profiles:
            return []

        segments = []
        for spec in profiles[profile_name].get("obstacles", []):
            waypoints = [self._waypoint_xy(item) for item in spec.get("waypoints", [])]
            if len(waypoints) < 2:
                continue
            clearance = self._obstacle_radius(spec) + self.dynamic_route_inflation
            pairs = list(zip(waypoints, waypoints[1:]))
            if str(spec.get("loop", "ping_pong")) == "cycle":
                pairs.append((waypoints[-1], waypoints[0]))
            for start, end in pairs:
                segments.append((start[0], start[1], end[0], end[1], clearance, str(spec.get("name", "dynamic_obstacle"))))
        if segments:
            rospy.loginfo("Loaded %d dynamic obstacle route segments for safe goal checks", len(segments))
        return segments

    @staticmethod
    def _distance_to_segment(px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            return math.hypot(px - x1, py - y1)
        ratio = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
        closest_x = x1 + ratio * dx
        closest_y = y1 + ratio * dy
        return math.hypot(px - closest_x, py - closest_y)

    @staticmethod
    def _orientation(ax, ay, bx, by, cx, cy):
        value = (by - ay) * (cx - bx) - (bx - ax) * (cy - by)
        if abs(value) <= 1e-9:
            return 0
        return 1 if value > 0.0 else 2

    @staticmethod
    def _on_segment(ax, ay, bx, by, cx, cy):
        return (
            min(ax, cx) - 1e-9 <= bx <= max(ax, cx) + 1e-9
            and min(ay, cy) - 1e-9 <= by <= max(ay, cy) + 1e-9
        )

    @classmethod
    def _segments_intersect(cls, ax, ay, bx, by, cx, cy, dx, dy):
        o1 = cls._orientation(ax, ay, bx, by, cx, cy)
        o2 = cls._orientation(ax, ay, bx, by, dx, dy)
        o3 = cls._orientation(cx, cy, dx, dy, ax, ay)
        o4 = cls._orientation(cx, cy, dx, dy, bx, by)
        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and cls._on_segment(ax, ay, cx, cy, bx, by):
            return True
        if o2 == 0 and cls._on_segment(ax, ay, dx, dy, bx, by):
            return True
        if o3 == 0 and cls._on_segment(cx, cy, ax, ay, dx, dy):
            return True
        if o4 == 0 and cls._on_segment(cx, cy, bx, by, dx, dy):
            return True
        return False

    @classmethod
    def _segment_to_segment_distance(cls, ax, ay, bx, by, cx, cy, dx, dy):
        if cls._segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
            return 0.0
        return min(
            cls._distance_to_segment(ax, ay, cx, cy, dx, dy),
            cls._distance_to_segment(bx, by, cx, cy, dx, dy),
            cls._distance_to_segment(cx, cy, ax, ay, bx, by),
            cls._distance_to_segment(dx, dy, ax, ay, bx, by),
        )

    def _current_pose(self, frame_id):
        try:
            translation, rotation = self._tf.lookupTransform(frame_id, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time(0)
        pose.pose.position.x = float(translation[0])
        pose.pose.position.y = float(translation[1])
        pose.pose.position.z = float(translation[2])
        pose.pose.orientation.x = rotation[0]
        pose.pose.orientation.y = rotation[1]
        pose.pose.orientation.z = rotation[2]
        pose.pose.orientation.w = rotation[3]
        return pose

    @staticmethod
    def _obstacle_center_and_radius(obstacle):
        points = list(obstacle.polygon.points)
        if not points:
            return None

        center_x = sum(point.x for point in points) / len(points)
        center_y = sum(point.y for point in points) / len(points)
        polygon_radius = max(math.hypot(point.x - center_x, point.y - center_y) for point in points)
        radius = max(float(obstacle.radius), polygon_radius)
        return center_x, center_y, max(0.0, radius)

    @staticmethod
    def _obstacle_velocity(obstacle):
        try:
            twist = obstacle.velocities.twist
            return float(twist.linear.x), float(twist.linear.y)
        except AttributeError:
            return 0.0, 0.0

    def _fresh_dynamic_obstacle_messages(self):
        if not self.use_dynamic_obstacle_predictions:
            return []

        now = time.monotonic()
        messages = []
        for message, wall_time in self._dynamic_obstacles.values():
            if now - wall_time <= self.dynamic_obstacle_prediction_timeout:
                messages.append(message)
        return messages

    def _path_to_pose_blocked_by_dynamic_obstacles(self, pose):
        messages = self._fresh_dynamic_obstacle_messages()
        if not messages:
            return False

        for message in messages:
            frame_id = message.header.frame_id or "map"
            target = self._transform_pose(pose, frame_id)
            start = self._current_pose(frame_id)
            if target is None or start is None:
                continue

            sx = start.pose.position.x
            sy = start.pose.position.y
            tx = target.pose.position.x
            ty = target.pose.position.y
            if math.hypot(tx - sx, ty - sy) <= 1e-6:
                continue

            for obstacle in message.obstacles:
                obstacle_shape = self._obstacle_center_and_radius(obstacle)
                if obstacle_shape is None:
                    continue
                ox, oy, radius = obstacle_shape
                vx, vy = self._obstacle_velocity(obstacle)
                px = ox + vx * self.dynamic_obstacle_prediction_horizon
                py = oy + vy * self.dynamic_obstacle_prediction_horizon
                distance = self._segment_to_segment_distance(sx, sy, tx, ty, ox, oy, px, py)
                clearance = radius + self.dynamic_obstacle_path_inflation
                if distance <= clearance:
                    if rospy.core.is_initialized():
                        rospy.logwarn_throttle(
                            2.0,
                            "Path to goal candidate (%.2f, %.2f) is %.2fm from predicted dynamic obstacle %d; required clearance %.2fm",
                            tx,
                            ty,
                            distance,
                            obstacle.id,
                            clearance,
                        )
                    return True
        return False

    def _is_on_dynamic_route(self, pose):
        if not self._dynamic_route_segments:
            return False

        route_pose = self._transform_pose(pose, "map")
        if route_pose is None:
            return False
        px = route_pose.pose.position.x
        py = route_pose.pose.position.y
        for x1, y1, x2, y2, clearance, name in self._dynamic_route_segments:
            distance = self._distance_to_segment(px, py, x1, y1, x2, y2)
            if distance <= clearance:
                if rospy.core.is_initialized():
                    rospy.logwarn_throttle(
                        2.0,
                        "Goal candidate (%.2f, %.2f) is %.2fm from dynamic route %s; required clearance %.2fm",
                        px,
                        py,
                        distance,
                        name,
                        clearance,
                    )
                return True
        return False

    def _is_pose_safe_in_grid(self, pose, grid, unknown_is_occupied=None):
        transformed = self._transform_pose(pose, grid.header.frame_id)
        if transformed is None:
            return None

        center = self._world_to_cell(
            grid,
            transformed.pose.position.x,
            transformed.pose.position.y,
        )
        if center is None:
            return None

        cx, cy = center
        resolution = grid.info.resolution
        cell_radius = max(0, int(math.ceil(self.target_check_radius / resolution)))
        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                if math.hypot(dx * resolution, dy * resolution) > self.target_check_radius:
                    continue
                mx = cx + dx
                my = cy + dy
                if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
                    return False
                value = grid.data[my * grid.info.width + mx]
                if self._cell_is_blocked(value, unknown_is_occupied):
                    return False
        return True

    def is_pose_safe(self, pose, unknown_is_occupied=None, check_dynamic_obstacles=True):
        if check_dynamic_obstacles:
            if self._is_on_dynamic_route(pose):
                return False
            if self._path_to_pose_blocked_by_dynamic_obstacles(pose):
                return False

        checked_any = False
        for grid in self._costmaps.values():
            safe = self._is_pose_safe_in_grid(pose, grid, unknown_is_occupied)
            if safe is None:
                continue
            checked_any = True
            if not safe:
                return False
        return checked_any

    def _candidate_pose(self, pose, dx, dy):
        candidate = PoseStamped()
        candidate.header.frame_id = pose.header.frame_id
        candidate.header.stamp = pose.header.stamp
        candidate.pose = copy.deepcopy(pose.pose)
        candidate.pose.position.x = pose.pose.position.x + dx
        candidate.pose.position.y = pose.pose.position.y + dy
        return candidate

    def find_nearest_safe_pose(self, pose, unknown_is_occupied=None, check_dynamic_obstacles=True):
        if self.search_step <= 0.0 or self.search_radius <= 0.0:
            return None

        ring_count = int(math.ceil(self.search_radius / self.search_step))
        for ring in range(1, ring_count + 1):
            radius = min(self.search_radius, ring * self.search_step)
            angle_count = max(16, int(math.ceil(2.0 * math.pi * radius / self.search_step)))
            for index in range(angle_count):
                angle = 2.0 * math.pi * index / angle_count
                candidate = self._candidate_pose(
                    pose,
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                )
                if self.is_pose_safe(candidate, unknown_is_occupied, check_dynamic_obstacles):
                    return candidate
        return None

    def _resolve_unknown_fallback(self, pose, name, waited, log_blocked):
        if not self.allow_unknown_fallback or not self.unknown_is_occupied:
            return None

        check_dynamic = self.unknown_fallback_use_dynamic_obstacle_predictions
        if self.is_pose_safe(pose, unknown_is_occupied=False, check_dynamic_obstacles=check_dynamic):
            if log_blocked:
                rospy.logwarn(
                    "Allowing %s at (%.2f, %.2f) through unknown map cells; move_base will attempt planning with allow_unknown",
                    name,
                    pose.pose.position.x,
                    pose.pose.position.y,
                )
            return ResolvedGoal(pose, waited=waited, reason="unknown_fallback")

        candidate = self.find_nearest_safe_pose(
            pose,
            unknown_is_occupied=False,
            check_dynamic_obstacles=check_dynamic,
        )
        if candidate is None:
            return None

        if log_blocked:
            rospy.logwarn(
                "Adjusted %s from (%.2f, %.2f) to unknown-space fallback pose (%.2f, %.2f)",
                name,
                pose.pose.position.x,
                pose.pose.position.y,
                candidate.pose.position.x,
                candidate.pose.position.y,
            )
        return ResolvedGoal(candidate, adjusted=True, waited=waited, reason="unknown_fallback_relocated")

    def resolve_pose(self, pose, name="goal", wait=True, log_blocked=True):
        self.wait_for_costmaps()
        if not self._costmaps:
            rospy.logwarn("No costmap is available; using requested %s unchanged", name)
            return ResolvedGoal(pose, reason="no_costmap")

        if self.is_pose_safe(pose):
            return ResolvedGoal(pose)

        waited = False
        if wait and self.wait_timeout > 0.0:
            rospy.logwarn(
                "Requested %s at (%.2f, %.2f) is occupied or unknown; waiting %.1fs before searching nearby",
                name,
                pose.pose.position.x,
                pose.pose.position.y,
                self.wait_timeout,
            )
            deadline = time.monotonic() + self.wait_timeout
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                time.sleep(self.wait_check_period)
                waited = True
                if self.is_pose_safe(pose):
                    rospy.loginfo("Requested %s became free after waiting", name)
                    return ResolvedGoal(pose, waited=waited, reason="became_free")

        candidate = self.find_nearest_safe_pose(pose)
        if candidate is None:
            fallback = self._resolve_unknown_fallback(pose, name, waited, log_blocked)
            if fallback is not None:
                return fallback

            if log_blocked:
                rospy.logerr(
                    "No safe replacement found for %s within %.2fm; rejecting unsafe target",
                    name,
                    self.search_radius,
                )
            return ResolvedGoal(pose, waited=waited, reason="no_replacement", blocked=True)

        if log_blocked:
            rospy.logwarn(
                "Adjusted %s from (%.2f, %.2f) to nearest safe pose (%.2f, %.2f)",
                name,
                pose.pose.position.x,
                pose.pose.position.y,
                candidate.pose.position.x,
                candidate.pose.position.y,
            )
        return ResolvedGoal(candidate, adjusted=True, waited=waited, reason="relocated")
