#!/usr/bin/env python3

import copy
import math
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from actionlib_msgs.msg import GoalID
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

        self._front = DirectionalScanState()
        self._rear = DirectionalScanState()
        self._scan_angle_span = 0.0
        self._emergency_active = False

        self._publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        self._cancel_publisher = rospy.Publisher(self.cancel_topic, GoalID, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber(self.input_topic, AckermannDriveStamped, self._cmd_callback, queue_size=1)
        rospy.loginfo(
            "ackermann safety supervisor forwarding %s to %s using %s",
            self.input_topic,
            self.output_topic,
            self.scan_topic,
        )

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
        if self._should_emergency_stop(message):
            self._publish_emergency_stop()
            return

        command = self._limit_reverse_speed(self._apply_slowdown(message))
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
