#!/usr/bin/env python3

import math
import threading

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _signed(value, sign_source):
    if sign_source < 0.0:
        return -abs(value)
    return abs(value)


class R1AckermannController:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/ackermann_cmd")
        self.wheelbase = float(rospy.get_param("~wheelbase", 0.41893))
        self.front_track = float(rospy.get_param("~front_track", 0.26462))
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.04075))
        self.max_speed = abs(float(rospy.get_param("~max_speed", 0.22)))
        self.max_steering_angle = abs(float(rospy.get_param("~max_steering_angle", 0.45)))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.45))
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))
        self.hold_steering_on_timeout = bool(rospy.get_param("~hold_steering_on_timeout", False))

        self.left_wheel_sign = float(rospy.get_param("~left_wheel_speed_sign", 1.0))
        self.right_wheel_sign = float(rospy.get_param("~right_wheel_speed_sign", -1.0))
        self.left_steering_sign = float(rospy.get_param("~left_steering_sign", 1.0))
        self.right_steering_sign = float(rospy.get_param("~right_steering_sign", 1.0))

        self.left_steering_topic = rospy.get_param(
            "~left_steering_topic",
            "/r1/front_left_steering_position_controller/command",
        )
        self.right_steering_topic = rospy.get_param(
            "~right_steering_topic",
            "/r1/front_right_steering_position_controller/command",
        )
        self.left_wheel_topic = rospy.get_param(
            "~left_wheel_topic",
            "/r1/front_left_wheel_velocity_controller/command",
        )
        self.right_wheel_topic = rospy.get_param(
            "~right_wheel_topic",
            "/r1/front_right_wheel_velocity_controller/command",
        )

        self._lock = threading.Lock()
        self._last_command = AckermannDriveStamped()
        self._last_command_wall = 0.0

        self._left_steering_pub = rospy.Publisher(self.left_steering_topic, Float64, queue_size=1)
        self._right_steering_pub = rospy.Publisher(self.right_steering_topic, Float64, queue_size=1)
        self._left_wheel_pub = rospy.Publisher(self.left_wheel_topic, Float64, queue_size=1)
        self._right_wheel_pub = rospy.Publisher(self.right_wheel_topic, Float64, queue_size=1)

        rospy.Subscriber(self.input_topic, AckermannDriveStamped, self._command_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._timer_callback)
        rospy.loginfo("R1 Ackermann controller consuming %s", self.input_topic)

    def _command_callback(self, message):
        with self._lock:
            self._last_command = message
            self._last_command_wall = rospy.get_time()

    def _front_wheel_angle(self, rear_center_radius, lateral_offset):
        angle = math.atan2(self.wheelbase, rear_center_radius - lateral_offset)
        if angle > math.pi / 2.0:
            angle -= math.pi
        elif angle < -math.pi / 2.0:
            angle += math.pi
        return angle

    def _front_wheel_linear_speed(self, speed, curvature, rear_center_radius, lateral_offset):
        if abs(curvature) <= 1e-6:
            return speed
        yaw_rate = abs(speed * curvature)
        path_radius = math.hypot(self.wheelbase, rear_center_radius - lateral_offset)
        return _signed(yaw_rate * path_radius, speed)

    def _targets_from_command(self, command, fresh):
        speed = _clamp(command.drive.speed, -self.max_speed, self.max_speed) if fresh else 0.0
        steering = _clamp(
            command.drive.steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )
        if not fresh and not self.hold_steering_on_timeout:
            steering = 0.0

        if abs(steering) <= 1e-5:
            left_steering = 0.0
            right_steering = 0.0
            left_linear_speed = speed
            right_linear_speed = speed
        else:
            curvature = math.tan(steering) / self.wheelbase
            rear_center_radius = 1.0 / curvature
            half_track = self.front_track / 2.0
            left_steering = self._front_wheel_angle(rear_center_radius, half_track)
            right_steering = self._front_wheel_angle(rear_center_radius, -half_track)
            left_linear_speed = self._front_wheel_linear_speed(
                speed,
                curvature,
                rear_center_radius,
                half_track,
            )
            right_linear_speed = self._front_wheel_linear_speed(
                speed,
                curvature,
                rear_center_radius,
                -half_track,
            )

        left_wheel_velocity = self.left_wheel_sign * left_linear_speed / self.wheel_radius
        right_wheel_velocity = self.right_wheel_sign * right_linear_speed / self.wheel_radius
        return (
            self.left_steering_sign * left_steering,
            self.right_steering_sign * right_steering,
            left_wheel_velocity,
            right_wheel_velocity,
        )

    def _timer_callback(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.get_time()
        with self._lock:
            command = self._last_command
            fresh = self._last_command_wall > 0.0 and now - self._last_command_wall <= self.command_timeout

        left_steering, right_steering, left_wheel, right_wheel = self._targets_from_command(command, fresh)
        try:
            self._left_steering_pub.publish(Float64(left_steering))
            self._right_steering_pub.publish(Float64(right_steering))
            self._left_wheel_pub.publish(Float64(left_wheel))
            self._right_wheel_pub.publish(Float64(right_wheel))
        except rospy.ROSException:
            return


def main():
    rospy.init_node("r1_ackermann_controller")
    R1AckermannController()
    rospy.spin()


if __name__ == "__main__":
    main()
