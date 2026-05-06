#!/usr/bin/env python3

import copy
import math
import time

import rospy
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from actionlib_msgs.msg import GoalID
from costmap_converter.msg import ObstacleArrayMsg
from sensor_msgs.msg import LaserScan


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _command_nonzero(message):
    return abs(message.drive.speed) > 1e-4


def _zero_command(frame_id):
    command = AckermannDriveStamped()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = frame_id
    return command


class DirectionalScanState:
    def __init__(self):
        self.minimum = float("inf")
        self.closing_speed = 0.0
        self.last_scan_wall = 0.0


class AckermannCmdSafetySupervisor:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/ackermann_cmd_safety_in")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.front_angle = float(rospy.get_param("~front_angle", 0.65))
        self.rear_angle = float(rospy.get_param("~rear_angle", self.front_angle))
        self.require_full_scan_for_reverse = bool(
            rospy.get_param("~require_full_scan_for_reverse", True)
        )
        self.min_full_scan_angle = float(rospy.get_param("~min_full_scan_angle", 6.0))
        self.hard_stop_distance = float(rospy.get_param("~hard_stop_distance", 0.45))
        self.slowdown_distance = float(rospy.get_param("~slowdown_distance", 0.85))
        self.reaction_time = float(rospy.get_param("~reaction_time", 0.35))
        self.max_deceleration = max(0.05, float(rospy.get_param("~max_deceleration", 0.45)))
        self.max_reverse_speed = abs(float(rospy.get_param("~max_reverse_speed", 0.08)))
        self.ttc_stop_time = float(rospy.get_param("~ttc_stop_time", 1.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.5))
        self.cancel_on_emergency = bool(rospy.get_param("~cancel_on_emergency", False))
        self.enable_dynamic_escape_boost = bool(rospy.get_param("~enable_dynamic_escape_boost", True))
        self.dynamic_obstacle_topic = rospy.get_param(
            "~dynamic_obstacle_topic",
            "/move_base/TebLocalPlannerROS/obstacles",
        )
        self.dynamic_obstacle_timeout = float(rospy.get_param("~dynamic_obstacle_timeout", 0.8))
        self.escape_prediction_horizon = max(
            0.0,
            float(rospy.get_param("~escape_prediction_horizon", 1.2)),
        )
        self.escape_threat_radius = max(0.0, float(rospy.get_param("~escape_threat_radius", 0.55)))
        self.escape_speed = abs(float(rospy.get_param("~escape_speed", 0.30)))
        self.escape_min_forward_clearance = max(
            0.0,
            float(rospy.get_param("~escape_min_forward_clearance", 1.05)),
        )
        self.escape_min_obstacle_speed = max(
            0.0,
            float(rospy.get_param("~escape_min_obstacle_speed", 0.10)),
        )

        self._front = DirectionalScanState()
        self._rear = DirectionalScanState()
        self._scan_angle_span = 0.0
        self._emergency_active = False
        self._dynamic_obstacle_message = None
        self._dynamic_obstacle_wall = 0.0
        self._tf = tf.TransformListener() if self.enable_dynamic_escape_boost else None

        self._publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        self._cancel_publisher = rospy.Publisher(self.cancel_topic, GoalID, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        if self.enable_dynamic_escape_boost:
            rospy.Subscriber(
                self.dynamic_obstacle_topic,
                ObstacleArrayMsg,
                self._dynamic_obstacle_callback,
                queue_size=1,
            )
        rospy.Subscriber(self.input_topic, AckermannDriveStamped, self._cmd_callback, queue_size=1)
        rospy.loginfo(
            "ackermann safety supervisor forwarding %s to %s using %s",
            self.input_topic,
            self.output_topic,
            self.scan_topic,
        )

    def _dynamic_obstacle_callback(self, message):
        self._dynamic_obstacle_message = message
        self._dynamic_obstacle_wall = time.monotonic()

    def _scan_callback(self, message):
        front_ranges = []
        rear_ranges = []
        for index, value in enumerate(message.ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= self.front_angle:
                front_ranges.append(value)
            if abs(abs(angle) - math.pi) <= self.rear_angle:
                rear_ranges.append(value)

        now = time.monotonic()
        self._scan_angle_span = abs(message.angle_max - message.angle_min)
        self._update_directional_state(self._front, front_ranges, now)
        self._update_directional_state(self._rear, rear_ranges, now)

    @staticmethod
    def _update_directional_state(state, ranges, now):
        if not ranges:
            state.minimum = float("inf")
            state.closing_speed = 0.0
            state.last_scan_wall = now
            return

        current_min = min(ranges)
        previous_min = state.minimum
        previous_wall = state.last_scan_wall
        state.minimum = current_min
        state.last_scan_wall = now

        if math.isfinite(previous_min) and previous_wall > 0.0 and now > previous_wall:
            state.closing_speed = max(0.0, (previous_min - current_min) / (now - previous_wall))
        else:
            state.closing_speed = 0.0

    def _scan_is_fresh(self):
        return self._state_is_fresh(self._front)

    def _state_is_fresh(self, state):
        return state.last_scan_wall > 0.0 and time.monotonic() - state.last_scan_wall <= self.scan_timeout

    def _has_full_scan_for_reverse(self):
        return self._scan_angle_span >= self.min_full_scan_angle

    def _required_stop_distance(self, speed):
        return self.hard_stop_distance + speed * self.reaction_time + (speed * speed) / (2.0 * self.max_deceleration)

    @staticmethod
    def _distance_to_segment(px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            return math.hypot(px - x1, py - y1), 0.0
        ratio = _clamp(((px - x1) * dx + (py - y1) * dy) / length_sq, 0.0, 1.0)
        closest_x = x1 + ratio * dx
        closest_y = y1 + ratio * dy
        return math.hypot(px - closest_x, py - closest_y), ratio

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

    def _robot_xy_in_frame(self, frame_id):
        if not frame_id or frame_id == self.frame_id:
            return 0.0, 0.0
        if self._tf is None:
            return None
        try:
            translation, _rotation = self._tf.lookupTransform(frame_id, self.frame_id, rospy.Time(0))
            return float(translation[0]), float(translation[1])
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Cannot evaluate dynamic escape boost without transform %s -> %s: %s",
                frame_id,
                self.frame_id,
                exc,
            )
            return None

    def _fresh_dynamic_obstacle_message(self):
        if not self.enable_dynamic_escape_boost or self._dynamic_obstacle_message is None:
            return None
        if time.monotonic() - self._dynamic_obstacle_wall > self.dynamic_obstacle_timeout:
            return None
        return self._dynamic_obstacle_message

    def _front_is_clear_for_escape(self, speed):
        if not self._state_is_fresh(self._front):
            return False
        required = max(
            self.escape_min_forward_clearance,
            self._required_stop_distance(abs(speed)),
        )
        return self._front.minimum >= required

    def _dynamic_escape_threat(self):
        message = self._fresh_dynamic_obstacle_message()
        if message is None or self.escape_prediction_horizon <= 0.0:
            return None

        frame_id = message.header.frame_id or self.frame_id
        robot_xy = self._robot_xy_in_frame(frame_id)
        if robot_xy is None:
            return None
        robot_x, robot_y = robot_xy

        for obstacle in message.obstacles:
            shape = self._obstacle_center_and_radius(obstacle)
            if shape is None:
                continue
            obstacle_x, obstacle_y, radius = shape
            velocity_x, velocity_y = self._obstacle_velocity(obstacle)
            obstacle_speed = math.hypot(velocity_x, velocity_y)
            if obstacle_speed < self.escape_min_obstacle_speed:
                continue

            toward_robot = (robot_x - obstacle_x) * velocity_x + (robot_y - obstacle_y) * velocity_y
            if toward_robot < 0.0:
                continue

            predicted_x = obstacle_x + velocity_x * self.escape_prediction_horizon
            predicted_y = obstacle_y + velocity_y * self.escape_prediction_horizon
            distance, _ratio = self._distance_to_segment(
                robot_x,
                robot_y,
                obstacle_x,
                obstacle_y,
                predicted_x,
                predicted_y,
            )
            clearance = radius + self.escape_threat_radius
            if distance <= clearance:
                return obstacle.id, distance, clearance
        return None

    def _apply_dynamic_escape_boost(self, command):
        if not self.enable_dynamic_escape_boost:
            return command
        if command.drive.speed <= 1e-4:
            return command
        if command.drive.speed >= self.escape_speed:
            return command
        if not self._front_is_clear_for_escape(self.escape_speed):
            return command

        threat = self._dynamic_escape_threat()
        if threat is None:
            return command

        obstacle_id, distance, clearance = threat
        boosted = copy.deepcopy(command)
        boosted.drive.speed = self.escape_speed
        rospy.loginfo_throttle(
            1.0,
            "Dynamic obstacle %s is predicted %.2fm from the robot footprint corridor %.2fm; boosting forward speed to %.2fm/s",
            obstacle_id,
            distance,
            clearance,
            boosted.drive.speed,
        )
        return boosted

    def _time_to_collision(self, state, speed):
        closing_speed = max(0.0, speed, state.closing_speed)
        remaining_distance = state.minimum - self.hard_stop_distance
        if closing_speed <= 1e-3 or remaining_distance <= 0.0:
            return 0.0 if remaining_distance <= 0.0 else float("inf")
        return remaining_distance / closing_speed

    def _should_emergency_stop(self, command):
        direction = self._front if command.drive.speed >= 0.0 else self._rear
        direction_name = "front" if command.drive.speed >= 0.0 else "rear"
        moving_speed = abs(command.drive.speed)

        if moving_speed <= 1e-4:
            return False

        if command.drive.speed < -1e-4 and (
            self.require_full_scan_for_reverse and not self._has_full_scan_for_reverse()
        ):
            rospy.logwarn_throttle(
                2.0,
                "Stopping reverse command because %s does not provide full 360-degree coverage",
                self.scan_topic,
            )
            return True

        if not self._state_is_fresh(direction):
            if _command_nonzero(command):
                rospy.logwarn_throttle(2.0, "Stopping ackermann_cmd because %s is stale", self.scan_topic)
            return _command_nonzero(command)

        if direction.minimum <= self.hard_stop_distance:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: %s obstacle %.2fm <= hard stop %.2fm",
                direction_name,
                direction.minimum,
                self.hard_stop_distance,
            )
            return True

        required_distance = self._required_stop_distance(moving_speed)
        ttc = self._time_to_collision(direction, moving_speed)
        if direction.minimum <= required_distance:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: %s obstacle %.2fm <= required %.2fm",
                direction_name,
                direction.minimum,
                required_distance,
            )
            return True
        if ttc <= self.ttc_stop_time:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: %s obstacle TTC %.2fs <= %.2fs",
                direction_name,
                ttc,
                self.ttc_stop_time,
            )
            return True
        return False

    def _apply_slowdown(self, command):
        if abs(command.drive.speed) <= 1e-4:
            return command
        state = self._front if command.drive.speed > 0.0 else self._rear
        if not self._state_is_fresh(state):
            return command
        if state.minimum >= self.slowdown_distance:
            return command

        available = max(0.0, state.minimum - self.hard_stop_distance)
        span = max(0.05, self.slowdown_distance - self.hard_stop_distance)
        scale = max(0.0, min(1.0, available / span))
        limited = copy.deepcopy(command)
        limited.drive.speed *= scale
        return limited

    def _limit_reverse_speed(self, command):
        if command.drive.speed >= 0.0:
            return command
        limited = copy.deepcopy(command)
        limited.drive.speed = _clamp(command.drive.speed, -self.max_reverse_speed, 0.0)
        return limited

    def _publish_emergency_stop(self):
        self._publisher.publish(_zero_command(self.frame_id))
        if self.cancel_on_emergency and not self._emergency_active:
            self._cancel_publisher.publish(GoalID())
        self._emergency_active = True

    def _cmd_callback(self, message):
        command = self._apply_dynamic_escape_boost(message)
        if self._should_emergency_stop(command):
            self._publish_emergency_stop()
            return

        command = self._limit_reverse_speed(self._apply_slowdown(command))
        self._emergency_active = False
        command.header.stamp = rospy.Time.now()
        if not command.header.frame_id:
            command.header.frame_id = self.frame_id
        self._publisher.publish(command)


def main():
    rospy.init_node("ackermann_cmd_safety_supervisor")
    AckermannCmdSafetySupervisor()
    rospy.spin()


if __name__ == "__main__":
    main()
