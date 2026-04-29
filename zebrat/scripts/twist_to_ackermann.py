#!/usr/bin/env python3

import math

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TwistToAckermann:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd_nav")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.wheelbase = float(rospy.get_param("~wheelbase", 0.41893))
        self.max_speed = abs(float(rospy.get_param("~max_speed", 0.22)))
        self.max_steering_angle = abs(float(rospy.get_param("~max_steering_angle", 0.45)))
        self.planner_max_steering_angle = min(
            abs(float(rospy.get_param("~planner_max_steering_angle", self.max_steering_angle))),
            self.max_steering_angle,
        )
        self.min_turn_speed = abs(float(rospy.get_param("~min_turn_speed", 0.06)))
        self.angular_deadband = abs(float(rospy.get_param("~angular_deadband", 1e-3)))
        self.allow_rotate_crawl = bool(rospy.get_param("~allow_rotate_crawl", True))

        self.publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber(self.input_topic, Twist, self.callback, queue_size=1)
        rospy.loginfo("twist_to_ackermann forwarding %s to %s", self.input_topic, self.output_topic)

    def callback(self, message):
        speed = _clamp(message.linear.x, -self.max_speed, self.max_speed)
        yaw_rate = message.angular.z

        if abs(message.linear.y) > 1e-4:
            rospy.logwarn_throttle(2.0, "Ignoring cmd_vel linear.y %.3f for Ackermann drive", message.linear.y)

        steering = 0.0
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

        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = self.frame_id
        command.drive.speed = speed
        command.drive.steering_angle = steering
        self.publisher.publish(command)


def main():
    rospy.init_node("twist_to_ackermann")
    TwistToAckermann()
    rospy.spin()


if __name__ == "__main__":
    main()
