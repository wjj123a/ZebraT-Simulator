#!/usr/bin/env python3

import math
import threading

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


def _command_magnitude(message):
    return abs(message.drive.speed) + abs(message.drive.steering_angle)


def _zero_command(frame_id):
    command = AckermannDriveStamped()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = frame_id
    return command


class AckermannCmdArbiter:
    def __init__(self):
        self.nav_topic = rospy.get_param("~nav_topic", "/ackermann_cmd_nav")
        self.teleop_topic = rospy.get_param("~teleop_topic", "/ackermann_cmd_teleop")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")
        self.teleop_timeout = float(rospy.get_param("~teleop_timeout", 0.75))
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 0.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self._lock = threading.Lock()
        self._latest_nav = _zero_command(self.frame_id)
        self._latest_teleop = _zero_command(self.frame_id)
        self._last_nav_wall_time = 0.0
        self._last_teleop_wall_time = 0.0
        self._last_output = _zero_command(self.frame_id)
        self._have_last_output = False

        self._publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber(self.nav_topic, AckermannDriveStamped, self._nav_callback, queue_size=1)
        rospy.Subscriber(self.teleop_topic, AckermannDriveStamped, self._teleop_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._timer_callback)
        rospy.loginfo(
            "ackermann arbiter forwarding nav=%s teleop=%s to %s",
            self.nav_topic,
            self.teleop_topic,
            self.output_topic,
        )

    def _nav_callback(self, message):
        with self._lock:
            self._latest_nav = message
            self._last_nav_wall_time = rospy.get_time()

    def _teleop_callback(self, message):
        with self._lock:
            self._latest_teleop = message
            self._last_teleop_wall_time = rospy.get_time()

    def _select_command(self):
        now = rospy.get_time()
        with self._lock:
            teleop_active = (now - self._last_teleop_wall_time) <= self.teleop_timeout
            nav_active = (now - self._last_nav_wall_time) <= self.nav_timeout

            if teleop_active:
                return self._latest_teleop
            if nav_active:
                return self._latest_nav
            return _zero_command(self.frame_id)

    @staticmethod
    def _same_command(left, right):
        return (
            math.isclose(left.drive.speed, right.drive.speed, abs_tol=1e-6)
            and math.isclose(left.drive.steering_angle, right.drive.steering_angle, abs_tol=1e-6)
        )

    def _timer_callback(self, _event):
        command = self._select_command()
        command.header.stamp = rospy.Time.now()
        if not command.header.frame_id:
            command.header.frame_id = self.frame_id

        if (
            self._have_last_output
            and self._same_command(command, self._last_output)
            and _command_magnitude(command) == 0.0
        ):
            return

        self._publisher.publish(command)
        self._last_output = command
        self._have_last_output = True


def main():
    rospy.init_node("ackermann_cmd_arbiter")
    AckermannCmdArbiter()
    rospy.spin()


if __name__ == "__main__":
    main()
