#!/usr/bin/env python3

import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def _wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointExplorer:
    def __init__(self):
        self.goal_set = rospy.get_param("~goal_set", "exploration")
        self.goals = rospy.get_param(f"~{self.goal_set}_goals", [])
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel_teleop")
        self.startup_delay = float(rospy.get_param("~startup_delay", 6.0))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.35))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.25))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.20))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.50))
        self.linear_gain = float(rospy.get_param("~linear_gain", 0.25))
        self.angular_gain = float(rospy.get_param("~angular_gain", 1.2))
        self.front_stop_distance = float(rospy.get_param("~front_stop_distance", 0.70))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 70.0))

        if not self.goals:
          raise rospy.ROSInitException(f"No goals configured for set '{self.goal_set}'")

        self._publisher = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self._odom = None
        self._front_min = float("inf")
        self._left_min = float("inf")
        self._right_min = float("inf")

        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan_callback, queue_size=1)

    def _odom_callback(self, message):
        orientation = message.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self._odom = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw,
        )

    def _scan_callback(self, message):
        finite_ranges = []
        left_ranges = []
        right_ranges = []
        front_ranges = []

        for index, value in enumerate(message.ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = message.angle_min + index * message.angle_increment
            finite_ranges.append(value)
            if -0.45 <= angle <= 0.45:
                front_ranges.append(value)
            elif angle > 0.45:
                left_ranges.append(value)
            else:
                right_ranges.append(value)

        if front_ranges:
            self._front_min = min(front_ranges)
        if left_ranges:
            self._left_min = min(left_ranges)
        if right_ranges:
            self._right_min = min(right_ranges)

    def _publish_stop(self):
        self._publisher.publish(Twist())

    def _publish_command(self, linear_x, angular_z):
        command = Twist()
        command.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        command.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        self._publisher.publish(command)

    def _drive_to_goal(self, goal):
        deadline = time.monotonic() + float(goal.get("timeout", self.goal_timeout))
        name = goal.get("name", f"{goal['x']},{goal['y']}")

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._odom is None:
                self._publish_stop()
                rospy.sleep(0.1)
                continue

            x, y, yaw = self._odom
            dx = float(goal["x"]) - x
            dy = float(goal["y"]) - y
            distance = math.hypot(dx, dy)
            target_yaw = float(goal.get("yaw", math.atan2(dy, dx)))
            heading_error = _wrap_to_pi(math.atan2(dy, dx) - yaw) if distance > 1e-3 else 0.0
            final_yaw_error = _wrap_to_pi(target_yaw - yaw)

            if distance <= self.position_tolerance and abs(final_yaw_error) <= self.yaw_tolerance:
                self._publish_stop()
                rospy.loginfo("Reached exploration goal %s", name)
                return True

            if self._front_min < self.front_stop_distance:
                turn_direction = 1.0 if self._left_min >= self._right_min else -1.0
                self._publish_command(0.0, turn_direction * self.max_angular_speed * 0.7)
                rospy.sleep(0.1)
                continue

            if distance <= self.position_tolerance:
                self._publish_command(0.0, self.angular_gain * final_yaw_error)
            elif abs(heading_error) > 0.45:
                self._publish_command(0.0, self.angular_gain * heading_error)
            else:
                linear = min(self.max_linear_speed, self.linear_gain * distance)
                angular = self.angular_gain * heading_error
                self._publish_command(linear, angular)

            rospy.sleep(0.1)

        self._publish_stop()
        rospy.logwarn("Timed out while exploring toward %s", name)
        return False

    def run(self):
        rospy.loginfo("Waiting %.1fs before starting waypoint exploration", self.startup_delay)
        time.sleep(self.startup_delay)

        for goal in self.goals:
            self._drive_to_goal(goal)

        self._publish_stop()
        rospy.loginfo("Waypoint exploration finished")


def main():
    rospy.init_node("explore_waypoints")
    explorer = WaypointExplorer()
    explorer.run()


if __name__ == "__main__":
    main()
