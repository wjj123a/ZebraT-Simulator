#!/usr/bin/env python3

import math
import time

import rospy
from costmap_converter.msg import ObstacleArrayMsg
from costmap_converter.msg import ObstacleMsg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point32
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


def _yaw_from_orientation(orientation):
    _roll, _pitch, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    return yaw


def _point32(x, y, z=0.0):
    point = Point32()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _obstacle_radius(spec):
    if "radius" in spec:
        return max(0.01, float(spec["radius"]))
    size = spec.get("size")
    if size and len(size) >= 2:
        return 0.5 * math.hypot(float(size[0]), float(size[1]))
    return 0.24 if str(spec.get("type", "cart")) == "pedestrian" else 0.35


def _rectangle_points(x, y, yaw, length, width):
    half_length = 0.5 * float(length)
    half_width = 0.5 * float(width)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    corners = [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ]
    points = []
    for local_x, local_y in corners:
        points.append(
            _point32(
                x + local_x * cos_yaw - local_y * sin_yaw,
                y + local_x * sin_yaw + local_y * cos_yaw,
            )
        )
    return points


def build_obstacle_message(obstacle_id, spec, x, y, yaw, vx, vy):
    message = ObstacleMsg()
    message.id = int(obstacle_id)
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    message.orientation.x = qx
    message.orientation.y = qy
    message.orientation.z = qz
    message.orientation.w = qw
    message.velocities.twist.linear.x = float(vx)
    message.velocities.twist.linear.y = float(vy)

    size = spec.get("size")
    if size and len(size) >= 2 and str(spec.get("type", "cart")) != "pedestrian":
        message.polygon.points = _rectangle_points(x, y, yaw, float(size[0]), float(size[1]))
        message.radius = 0.0
    else:
        message.polygon.points = [_point32(x, y)]
        message.radius = _obstacle_radius(spec)
    return message


def _clamp_vector(vx, vy, max_speed):
    speed = math.hypot(vx, vy)
    if speed <= max_speed or speed <= 1e-6:
        return vx, vy
    scale = max_speed / speed
    return vx * scale, vy * scale


class TrackedObstacle:
    def __init__(self, obstacle_id, spec, speed_scale, velocity_smoothing_alpha):
        self.obstacle_id = obstacle_id
        self.spec = spec
        self.name = str(spec["name"])
        self.nominal_speed = max(0.0, float(spec.get("speed", 0.0)) * speed_scale)
        self.velocity_smoothing_alpha = min(1.0, max(0.0, velocity_smoothing_alpha))
        self.x = None
        self.y = None
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_wall_time = None
        self.last_seen_wall_time = None

    def _nominal_velocity(self, yaw):
        return self.nominal_speed * math.cos(yaw), self.nominal_speed * math.sin(yaw)

    def update(self, pose, twist, wall_time):
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = _yaw_from_orientation(pose.orientation)
        measured_vx = float(twist.linear.x) if twist is not None else 0.0
        measured_vy = float(twist.linear.y) if twist is not None else 0.0
        has_twist = math.hypot(measured_vx, measured_vy) > 1e-4

        if not has_twist and self.x is not None and self.last_wall_time is not None:
            dt = wall_time - self.last_wall_time
            if dt >= 1e-3:
                dx = x - self.x
                dy = y - self.y
                if math.hypot(dx, dy) > 1e-4:
                    measured_vx = dx / dt
                    measured_vy = dy / dt
                    has_twist = True

        if not has_twist:
            measured_vx, measured_vy = self._nominal_velocity(yaw)

        max_speed = max(0.10, self.nominal_speed * 2.5)
        measured_vx, measured_vy = _clamp_vector(measured_vx, measured_vy, max_speed)
        alpha = self.velocity_smoothing_alpha
        if self.last_wall_time is None:
            self.vx = measured_vx
            self.vy = measured_vy
        else:
            self.vx = alpha * measured_vx + (1.0 - alpha) * self.vx
            self.vy = alpha * measured_vy + (1.0 - alpha) * self.vy

        self.x = x
        self.y = y
        self.yaw = yaw
        self.last_wall_time = wall_time
        self.last_seen_wall_time = wall_time

    def is_fresh(self, wall_time, timeout):
        return self.last_seen_wall_time is not None and wall_time - self.last_seen_wall_time <= timeout

    def to_message(self):
        return build_obstacle_message(self.obstacle_id, self.spec, self.x, self.y, self.yaw, self.vx, self.vy)


class DynamicObstaclePredictionPublisher:
    def __init__(self):
        self.profile_name = rospy.get_param(
            "~profile",
            rospy.get_param("/dynamic_obstacles/profile", "area_mixed"),
        )
        self.speed_scale = float(
            rospy.get_param("~speed_scale", rospy.get_param("/dynamic_obstacles/speed_scale", 1.0))
        )
        profiles = rospy.get_param("~profiles", rospy.get_param("/dynamic_obstacles/profiles", {}))
        if self.profile_name not in profiles:
            raise rospy.ROSInitException("Dynamic obstacle profile '%s' was not found" % self.profile_name)

        self.profile = profiles[self.profile_name]
        self.output_topic = rospy.get_param("~output_topic", "/move_base/TebLocalPlannerROS/obstacles")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", self.profile.get("update_rate", 12.0)))
        self.model_timeout = float(rospy.get_param("~model_timeout", 1.0))
        velocity_alpha = float(rospy.get_param("~velocity_smoothing_alpha", 0.45))

        self.trackers = {}
        for index, spec in enumerate(self.profile.get("obstacles", []), start=1):
            name = str(spec["name"])
            self.trackers[name] = TrackedObstacle(index, spec, self.speed_scale, velocity_alpha)
        if not self.trackers:
            raise rospy.ROSInitException("Dynamic obstacle profile '%s' has no obstacles" % self.profile_name)

        self.publisher = rospy.Publisher(self.output_topic, ObstacleArrayMsg, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self._model_states_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.publish_rate)), self._publish_timer)
        rospy.loginfo(
            "Publishing %d dynamic obstacle predictions from %s to %s",
            len(self.trackers),
            self.model_states_topic,
            self.output_topic,
        )

    def _model_states_callback(self, message):
        wall_time = time.monotonic()
        for index, name in enumerate(message.name):
            tracker = self.trackers.get(name)
            if tracker is None:
                continue
            twist = message.twist[index] if index < len(message.twist) else None
            tracker.update(message.pose[index], twist, wall_time)

    def _publish_timer(self, _event):
        now = time.monotonic()
        message = ObstacleArrayMsg()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        for tracker in self.trackers.values():
            if tracker.is_fresh(now, self.model_timeout):
                message.obstacles.append(tracker.to_message())
        self.publisher.publish(message)


def main():
    rospy.init_node("dynamic_obstacle_predictions")
    DynamicObstaclePredictionPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
