#!/usr/bin/env python3

import copy
import math
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import LaserScan


def _command_nonzero(message):
    return abs(message.drive.speed) > 1e-4


def _zero_command(frame_id):
    command = AckermannDriveStamped()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = frame_id
    return command


class AckermannCmdSafetySupervisor:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/ackermann_cmd_safety_in")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.front_angle = float(rospy.get_param("~front_angle", 0.65))
        self.hard_stop_distance = float(rospy.get_param("~hard_stop_distance", 0.45))
        self.slowdown_distance = float(rospy.get_param("~slowdown_distance", 0.85))
        self.reaction_time = float(rospy.get_param("~reaction_time", 0.35))
        self.max_deceleration = max(0.05, float(rospy.get_param("~max_deceleration", 0.45)))
        self.ttc_stop_time = float(rospy.get_param("~ttc_stop_time", 1.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.5))
        self.cancel_on_emergency = bool(rospy.get_param("~cancel_on_emergency", False))

        self._front_min = float("inf")
        self._closing_speed = 0.0
        self._last_scan_wall = 0.0
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
        ranges = []
        for index, value in enumerate(message.ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= self.front_angle:
                ranges.append(value)

        now = time.monotonic()
        if not ranges:
            self._front_min = float("inf")
            self._closing_speed = 0.0
            self._last_scan_wall = now
            return

        current_min = min(ranges)
        previous_min = self._front_min
        previous_wall = self._last_scan_wall
        self._front_min = current_min
        self._last_scan_wall = now

        if math.isfinite(previous_min) and previous_wall > 0.0 and now > previous_wall:
            self._closing_speed = max(0.0, (previous_min - current_min) / (now - previous_wall))
        else:
            self._closing_speed = 0.0

    def _scan_is_fresh(self):
        return self._last_scan_wall > 0.0 and time.monotonic() - self._last_scan_wall <= self.scan_timeout

    def _required_stop_distance(self, speed):
        return self.hard_stop_distance + speed * self.reaction_time + (speed * speed) / (2.0 * self.max_deceleration)

    def _time_to_collision(self, speed):
        closing_speed = max(0.0, speed, self._closing_speed)
        remaining_distance = self._front_min - self.hard_stop_distance
        if closing_speed <= 1e-3 or remaining_distance <= 0.0:
            return 0.0 if remaining_distance <= 0.0 else float("inf")
        return remaining_distance / closing_speed

    def _should_emergency_stop(self, command):
        if not self._scan_is_fresh():
            if _command_nonzero(command):
                rospy.logwarn_throttle(2.0, "Stopping ackermann_cmd because %s is stale", self.scan_topic)
            return _command_nonzero(command)

        forward_speed = max(0.0, command.drive.speed)
        if forward_speed <= 1e-4:
            return False

        if self._front_min <= self.hard_stop_distance:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: front obstacle %.2fm <= hard stop %.2fm",
                self._front_min,
                self.hard_stop_distance,
            )
            return True

        required_distance = self._required_stop_distance(forward_speed)
        ttc = self._time_to_collision(forward_speed)
        if self._front_min <= required_distance:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: front obstacle %.2fm <= required %.2fm",
                self._front_min,
                required_distance,
            )
            return True
        if ttc <= self.ttc_stop_time:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: front obstacle TTC %.2fs <= %.2fs",
                ttc,
                self.ttc_stop_time,
            )
            return True
        return False

    def _apply_slowdown(self, command):
        if command.drive.speed <= 0.0 or not self._scan_is_fresh():
            return command
        if self._front_min >= self.slowdown_distance:
            return command

        available = max(0.0, self._front_min - self.hard_stop_distance)
        span = max(0.05, self.slowdown_distance - self.hard_stop_distance)
        scale = max(0.0, min(1.0, available / span))
        limited = copy.deepcopy(command)
        limited.drive.speed *= scale
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

        command = self._apply_slowdown(message)
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
