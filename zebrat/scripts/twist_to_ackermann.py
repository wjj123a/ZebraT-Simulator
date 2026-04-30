#!/usr/bin/env python3

import math
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import tf2_ros
from tf.transformations import quaternion_matrix


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TwistToAckermann:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd_nav")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")
        self.angular_input_mode = rospy.get_param("~angular_input_mode", "yaw_rate")

        self.wheelbase = float(rospy.get_param("~wheelbase", 0.41893))
        self.max_speed = abs(float(rospy.get_param("~max_speed", 0.22)))
        self.max_reverse_speed = abs(float(rospy.get_param("~max_reverse_speed", self.max_speed)))
        self.allow_reverse = bool(rospy.get_param("~allow_reverse", False))
        self.suppressed_reverse_behavior = rospy.get_param("~suppressed_reverse_behavior", "stop")
        self.max_steering_angle = abs(float(rospy.get_param("~max_steering_angle", 0.45)))
        self.planner_max_steering_angle = min(
            abs(float(rospy.get_param("~planner_max_steering_angle", self.max_steering_angle))),
            self.max_steering_angle,
        )
        self.min_turn_speed = abs(float(rospy.get_param("~min_turn_speed", 0.06)))
        self.angular_deadband = abs(float(rospy.get_param("~angular_deadband", 1e-3)))
        self.allow_rotate_crawl = bool(rospy.get_param("~allow_rotate_crawl", True))
        self.steering_rate_limit = abs(float(rospy.get_param("~steering_rate_limit", 0.45)))
        self.steering_smoothing_alpha = _clamp(
            float(rospy.get_param("~steering_smoothing_alpha", 0.35)),
            0.0,
            1.0,
        )
        self.steering_deadband = abs(float(rospy.get_param("~steering_deadband", 0.025)))
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.global_plan_topic = rospy.get_param("~global_plan_topic", "/move_base/GlobalPlanner/plan")
        self.forward_assist_speed = abs(
            float(rospy.get_param("~forward_assist_speed", self.min_turn_speed))
        )
        self.forward_assist_carrot_dist = abs(
            float(rospy.get_param("~forward_assist_carrot_dist", 1.0))
        )
        self.forward_assist_min_carrot_dist = abs(
            float(rospy.get_param("~forward_assist_min_carrot_dist", 0.35))
        )
        self.forward_assist_max_age = abs(
            float(rospy.get_param("~forward_assist_max_age", 2.0))
        )
        self.forward_assist_steering_gain = abs(
            float(rospy.get_param("~forward_assist_steering_gain", 1.0))
        )
        self._last_steering = None
        self._last_wall = None
        self._latest_goal = None
        self._latest_goal_wall = 0.0
        self._latest_plan = None
        self._latest_plan_wall = 0.0
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        if self.angular_input_mode not in ("yaw_rate", "steering_angle"):
            rospy.logwarn(
                "Unknown angular_input_mode '%s'; falling back to yaw_rate",
                self.angular_input_mode,
            )
            self.angular_input_mode = "yaw_rate"
        if self.suppressed_reverse_behavior not in ("stop", "forward_crawl"):
            rospy.logwarn(
                "Unknown suppressed_reverse_behavior '%s'; falling back to stop",
                self.suppressed_reverse_behavior,
            )
            self.suppressed_reverse_behavior = "stop"

        self.publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber(self.input_topic, Twist, self.callback, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback, queue_size=1)
        rospy.Subscriber(self.global_plan_topic, Path, self._plan_callback, queue_size=1)
        rospy.loginfo("twist_to_ackermann forwarding %s to %s", self.input_topic, self.output_topic)

    def _goal_callback(self, message):
        self._latest_goal = message
        self._latest_goal_wall = time.monotonic()

    def _plan_callback(self, message):
        self._latest_plan = message
        self._latest_plan_wall = time.monotonic()

    def callback(self, message):
        speed, reverse_suppressed = self._clamp_speed(message.linear.x)
        if reverse_suppressed:
            rospy.logwarn_throttle(
                2.0,
                "Suppressing planner reverse command %.3f; reverse is reserved for recovery",
                message.linear.x,
            )
        angular_value = message.angular.z

        if abs(message.linear.y) > 1e-4:
            rospy.logwarn_throttle(2.0, "Ignoring cmd_vel linear.y %.3f for Ackermann drive", message.linear.y)

        steering = 0.0
        if self.angular_input_mode == "steering_angle":
            steering = _clamp(
                angular_value,
                -self.planner_max_steering_angle,
                self.planner_max_steering_angle,
            )
        else:
            yaw_rate = angular_value
            if abs(speed) <= 1e-4:
                if self.allow_rotate_crawl and abs(yaw_rate) > self.angular_deadband:
                    speed = self.min_turn_speed
                else:
                    yaw_rate = 0.0

            if abs(speed) > 1e-4 and abs(yaw_rate) > self.angular_deadband:
                steering = math.atan(self.wheelbase * yaw_rate / speed)
                steering = _clamp(
                    steering,
                    -self.planner_max_steering_angle,
                    self.planner_max_steering_angle,
                )

        if reverse_suppressed and self.suppressed_reverse_behavior == "stop":
            steering = 0.0
        elif reverse_suppressed and self.suppressed_reverse_behavior == "forward_crawl":
            assisted_steering = self._forward_turn_assist_steering(steering)
            if assisted_steering is not None:
                steering = assisted_steering
            if abs(steering) > self.steering_deadband:
                speed = _clamp(
                    max(self.forward_assist_speed, self.min_turn_speed),
                    0.0,
                    self.max_speed,
                )
                rospy.loginfo_throttle(
                    2.0,
                    "Planner requested reverse; using forward turn assist speed %.3f steering %.3f",
                    speed,
                    steering,
                )
            else:
                speed = 0.0
        elif self.angular_input_mode == "steering_angle" and abs(speed) <= 1e-4:
            if self.allow_rotate_crawl and abs(steering) > self.angular_deadband:
                speed = self.min_turn_speed
            else:
                steering = 0.0

        steering = self._smooth_steering(steering)

        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = self.frame_id
        command.drive.speed = speed
        command.drive.steering_angle = steering
        self.publisher.publish(command)

    def _clamp_speed(self, linear_x):
        if not self.allow_reverse and linear_x < 0.0:
            return 0.0, True
        return _clamp(linear_x, -self.max_reverse_speed, self.max_speed), False

    def _forward_turn_assist_steering(self, fallback_steering):
        carrot = self._plan_carrot_in_base()
        if carrot is None:
            carrot = self._goal_in_base()
        if carrot is None:
            return fallback_steering

        x, y = carrot
        if not math.isfinite(x) or not math.isfinite(y):
            return fallback_steering
        if math.hypot(x, y) < 1e-3:
            return None

        heading = math.atan2(y, x)
        if x < 0.05 and abs(y) < 0.05:
            if abs(fallback_steering) > self.steering_deadband:
                heading = math.copysign(math.pi / 2.0, fallback_steering)
            elif self._last_steering is not None and abs(self._last_steering) > self.steering_deadband:
                heading = math.copysign(math.pi / 2.0, self._last_steering)

        steering = self.forward_assist_steering_gain * heading
        return _clamp(
            steering,
            -self.planner_max_steering_angle,
            self.planner_max_steering_angle,
        )

    def _plan_carrot_in_base(self):
        plan = self._latest_plan
        if plan is None or not plan.poses:
            return None
        if time.monotonic() - self._latest_plan_wall > self.forward_assist_max_age:
            return None

        transforms = {}
        selected = None
        for pose in plan.poses:
            frame_id = pose.header.frame_id or plan.header.frame_id
            if not frame_id:
                continue
            if frame_id not in transforms:
                transform = self._lookup_to_base(frame_id)
                if transform is None:
                    continue
                transforms[frame_id] = transform

            point = self._transform_point(transforms[frame_id], pose.pose.position)
            if point is None:
                continue
            dist = math.hypot(point[0], point[1])
            if dist < self.forward_assist_min_carrot_dist:
                continue
            selected = point
            if dist >= self.forward_assist_carrot_dist:
                return point

        return selected

    def _goal_in_base(self):
        goal = self._latest_goal
        if goal is None:
            return None
        if time.monotonic() - self._latest_goal_wall > self.forward_assist_max_age:
            return None

        frame_id = goal.header.frame_id
        if not frame_id:
            return None
        transform = self._lookup_to_base(frame_id)
        if transform is None:
            return None
        return self._transform_point(transform, goal.pose.position)

    def _lookup_to_base(self, source_frame):
        try:
            return self._tf_buffer.lookup_transform(
                self.frame_id,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    @staticmethod
    def _transform_point(transform, point):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        x = matrix[0][0] * point.x + matrix[0][1] * point.y + matrix[0][2] * point.z + translation.x
        y = matrix[1][0] * point.x + matrix[1][1] * point.y + matrix[1][2] * point.z + translation.y
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return x, y

    def _smooth_steering(self, steering):
        if abs(steering) < self.steering_deadband:
            steering = 0.0

        now = time.monotonic()
        if self._last_steering is None or self._last_wall is None:
            self._last_steering = steering
            self._last_wall = now
            return steering

        dt = now - self._last_wall
        self._last_wall = now
        if dt <= 0.0 or dt > 1.0:
            self._last_steering = steering
            return steering

        blended = self._last_steering + self.steering_smoothing_alpha * (steering - self._last_steering)
        if self.steering_rate_limit > 0.0:
            max_step = self.steering_rate_limit * dt
            blended = _clamp(
                blended,
                self._last_steering - max_step,
                self._last_steering + max_step,
            )

        self._last_steering = blended
        return blended


def main():
    rospy.init_node("twist_to_ackermann")
    TwistToAckermann()
    rospy.spin()


if __name__ == "__main__":
    main()
