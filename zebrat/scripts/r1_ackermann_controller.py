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
        self.rear_track = float(rospy.get_param("~rear_track", 0.24182))
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.04075))
        self.drive_wheels = rospy.get_param("~drive_wheels", "all")
        self.max_speed = abs(float(rospy.get_param("~max_speed", 0.22)))
        self.max_reverse_speed = abs(float(rospy.get_param("~max_reverse_speed", self.max_speed)))
        self.max_steering_angle = abs(float(rospy.get_param("~max_steering_angle", 0.45)))
        self.steering_trim = float(rospy.get_param("~steering_trim", 0.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.45))
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))
        self.hold_steering_on_timeout = bool(rospy.get_param("~hold_steering_on_timeout", False))

        self.left_wheel_sign = float(rospy.get_param("~left_wheel_speed_sign", 1.0))
        self.right_wheel_sign = float(rospy.get_param("~right_wheel_speed_sign", -1.0))
        self.left_wheel_speed_scale = float(rospy.get_param("~left_wheel_speed_scale", 1.0))
        self.right_wheel_speed_scale = float(rospy.get_param("~right_wheel_speed_scale", 1.0))
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
        self.rear_left_wheel_topic = rospy.get_param(
            "~rear_left_wheel_topic",
            "/r1/rear_left_wheel_velocity_controller/command",
        )
        self.rear_right_wheel_topic = rospy.get_param(
            "~rear_right_wheel_topic",
            "/r1/rear_right_wheel_velocity_controller/command",
        )

        if self.drive_wheels not in ("front", "rear", "all"):
            rospy.logwarn(
                "Unknown drive_wheels '%s'; falling back to all",
                self.drive_wheels,
            )
            self.drive_wheels = "all"

        self._lock = threading.Lock()
        self._last_command = AckermannDriveStamped()
        self._last_command_wall = 0.0

        self._left_steering_pub = rospy.Publisher(
            self.left_steering_topic, Float64, queue_size=1, latch=True
        )
        self._right_steering_pub = rospy.Publisher(
            self.right_steering_topic, Float64, queue_size=1, latch=True
        )
        self._left_wheel_pub = None
        self._right_wheel_pub = None
        self._rear_left_wheel_pub = None
        self._rear_right_wheel_pub = None
        if self.drive_wheels in ("front", "all"):
            self._left_wheel_pub = rospy.Publisher(
                self.left_wheel_topic, Float64, queue_size=1, latch=True
            )
            self._right_wheel_pub = rospy.Publisher(
                self.right_wheel_topic, Float64, queue_size=1, latch=True
            )
        if self.drive_wheels in ("rear", "all"):
            self._rear_left_wheel_pub = rospy.Publisher(
                self.rear_left_wheel_topic, Float64, queue_size=1, latch=True
            )
            self._rear_right_wheel_pub = rospy.Publisher(
                self.rear_right_wheel_topic, Float64, queue_size=1, latch=True
            )

        rospy.Subscriber(self.input_topic, AckermannDriveStamped, self._command_callback, queue_size=1)
        self._publish_targets(self._targets_from_command(AckermannDriveStamped(), False))
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

    def _rear_wheel_linear_speed(self, speed, curvature, rear_center_radius, lateral_offset):
        if abs(curvature) <= 1e-6:
            return speed
        yaw_rate = abs(speed * curvature)
        path_radius = abs(rear_center_radius - lateral_offset)
        return _signed(yaw_rate * path_radius, speed)

    def _targets_from_command(self, command, fresh):
        speed = _clamp(command.drive.speed, -self.max_reverse_speed, self.max_speed) if fresh else 0.0
        steering = _clamp(
            command.drive.steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )
        if not fresh and not self.hold_steering_on_timeout:
            steering = 0.0
        if fresh and abs(speed) > 1e-4 and abs(self.steering_trim) > 1e-6:
            steering = _clamp(
                steering + self.steering_trim,
                -self.max_steering_angle,
                self.max_steering_angle,
            )

        if abs(steering) <= 1e-5:
            left_steering = 0.0
            right_steering = 0.0
            left_front_linear_speed = speed
            right_front_linear_speed = speed
            left_rear_linear_speed = speed
            right_rear_linear_speed = speed
        else:
            curvature = math.tan(steering) / self.wheelbase
            rear_center_radius = 1.0 / curvature
            front_half_track = self.front_track / 2.0
            rear_half_track = self.rear_track / 2.0
            left_steering = self._front_wheel_angle(rear_center_radius, front_half_track)
            right_steering = self._front_wheel_angle(rear_center_radius, -front_half_track)
            left_front_linear_speed = self._front_wheel_linear_speed(
                speed,
                curvature,
                rear_center_radius,
                front_half_track,
            )
            right_front_linear_speed = self._front_wheel_linear_speed(
                speed,
                curvature,
                rear_center_radius,
                -front_half_track,
            )
            left_rear_linear_speed = self._rear_wheel_linear_speed(
                speed,
                curvature,
                rear_center_radius,
                rear_half_track,
            )
            right_rear_linear_speed = self._rear_wheel_linear_speed(
                speed,
                curvature,
                rear_center_radius,
                -rear_half_track,
            )

        left_front_wheel_velocity = (
            self.left_wheel_sign
            * self.left_wheel_speed_scale
            * left_front_linear_speed
            / self.wheel_radius
        )
        right_front_wheel_velocity = (
            self.right_wheel_sign
            * self.right_wheel_speed_scale
            * right_front_linear_speed
            / self.wheel_radius
        )
        left_rear_wheel_velocity = (
            self.left_wheel_sign
            * self.left_wheel_speed_scale
            * left_rear_linear_speed
            / self.wheel_radius
        )
        right_rear_wheel_velocity = (
            self.right_wheel_sign
            * self.right_wheel_speed_scale
            * right_rear_linear_speed
            / self.wheel_radius
        )
        return (
            self.left_steering_sign * left_steering,
            self.right_steering_sign * right_steering,
            left_front_wheel_velocity,
            right_front_wheel_velocity,
            left_rear_wheel_velocity,
            right_rear_wheel_velocity,
        )

    def _timer_callback(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.get_time()
        with self._lock:
            command = self._last_command
            fresh = self._last_command_wall > 0.0 and now - self._last_command_wall <= self.command_timeout

        (
            left_steering,
            right_steering,
            left_front_wheel,
            right_front_wheel,
            left_rear_wheel,
            right_rear_wheel,
        ) = self._targets_from_command(command, fresh)
        self._publish_targets(
            (
                left_steering,
                right_steering,
                left_front_wheel,
                right_front_wheel,
                left_rear_wheel,
                right_rear_wheel,
            )
        )

    def _publish_targets(self, targets):
        (
            left_steering,
            right_steering,
            left_front_wheel,
            right_front_wheel,
            left_rear_wheel,
            right_rear_wheel,
        ) = targets
        try:
            self._left_steering_pub.publish(Float64(left_steering))
            self._right_steering_pub.publish(Float64(right_steering))
            if self._left_wheel_pub is not None:
                self._left_wheel_pub.publish(Float64(left_front_wheel))
            if self._right_wheel_pub is not None:
                self._right_wheel_pub.publish(Float64(right_front_wheel))
            if self._rear_left_wheel_pub is not None:
                self._rear_left_wheel_pub.publish(Float64(left_rear_wheel))
            if self._rear_right_wheel_pub is not None:
                self._rear_right_wheel_pub.publish(Float64(right_rear_wheel))
        except rospy.ROSException:
            return


def main():
    rospy.init_node("r1_ackermann_controller")
    R1AckermannController()
    rospy.spin()


if __name__ == "__main__":
    main()
