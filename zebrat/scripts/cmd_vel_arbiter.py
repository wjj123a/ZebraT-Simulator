#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import Twist


def _twist_magnitude(message):
    return abs(message.linear.x) + abs(message.linear.y) + abs(message.angular.z)


class CmdVelArbiter:
    def __init__(self):
        self.nav_topic = rospy.get_param("~nav_topic", "/cmd_vel_nav")
        self.teleop_topic = rospy.get_param("~teleop_topic", "/cmd_vel_teleop")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.teleop_timeout = float(rospy.get_param("~teleop_timeout", 0.6))
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 0.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self._lock = threading.Lock()
        self._latest_nav = Twist()
        self._latest_teleop = Twist()
        self._last_nav_wall_time = 0.0
        self._last_teleop_wall_time = 0.0
        self._last_output = Twist()
        self._have_last_output = False

        self._publisher = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        rospy.Subscriber(self.nav_topic, Twist, self._nav_callback, queue_size=1)
        rospy.Subscriber(self.teleop_topic, Twist, self._teleop_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._timer_callback)

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
        zero = Twist()
        with self._lock:
            teleop_active = (now - self._last_teleop_wall_time) <= self.teleop_timeout
            nav_active = (now - self._last_nav_wall_time) <= self.nav_timeout

            if teleop_active:
                return self._latest_teleop
            if nav_active:
                return self._latest_nav
            return zero

    def _same_twist(self, left, right):
        return (
            math.isclose(left.linear.x, right.linear.x, abs_tol=1e-6)
            and math.isclose(left.linear.y, right.linear.y, abs_tol=1e-6)
            and math.isclose(left.angular.z, right.angular.z, abs_tol=1e-6)
        )

    def _timer_callback(self, _event):
        command = self._select_command()
        if (
            self._have_last_output
            and self._same_twist(command, self._last_output)
            and _twist_magnitude(command) == 0.0
        ):
            return

        self._publisher.publish(command)
        self._last_output = command
        self._have_last_output = True


def main():
    rospy.init_node("cmd_vel_arbiter")
    CmdVelArbiter()
    rospy.spin()


if __name__ == "__main__":
    main()
