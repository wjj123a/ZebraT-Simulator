#!/usr/bin/env python3

import math
import time
from collections import deque

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros
from tf.transformations import euler_from_quaternion


def _wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _pose_tuple(pose):
    orientation = pose.orientation
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    return float(pose.position.x), float(pose.position.y), float(yaw)


class AckermannReverseRecovery:
    def __init__(self):
        self.teleop_topic = rospy.get_param("~teleop_topic", "/ackermann_cmd_teleop")
        self.nav_command_topic = rospy.get_param("~nav_command_topic", "/ackermann_cmd_nav")
        self.raw_cmd_vel_topic = rospy.get_param("~raw_cmd_vel_topic", "/cmd_vel_nav")
        self.status_topic = rospy.get_param("~status_topic", "/move_base/status")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.clear_costmaps_service = rospy.get_param(
            "~clear_costmaps_service",
            "/move_base/clear_costmaps",
        )
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.reverse_speed = -abs(float(rospy.get_param("~reverse_speed", 0.06)))
        self.reverse_duration = float(rospy.get_param("~reverse_duration", 0.8))
        self.reverse_steering_angle = abs(float(rospy.get_param("~reverse_steering_angle", 0.35)))
        self.reverse_opposes_last_turn = bool(rospy.get_param("~reverse_opposes_last_turn", True))
        self.stop_duration = float(rospy.get_param("~stop_duration", 0.25))
        self.recovery_cooldown = float(rospy.get_param("~recovery_cooldown", 10.0))
        self.max_reverses_per_goal = int(rospy.get_param("~max_reverses_per_goal", 1))

        self.min_goal_age = float(rospy.get_param("~min_goal_age", 35.0))
        self.stuck_timeout = float(rospy.get_param("~stuck_timeout", 26.0))
        self.post_clear_wait = float(rospy.get_param("~post_clear_wait", 10.0))
        self.command_window = float(rospy.get_param("~command_window", 16.0))
        self.min_forward_attempts = int(rospy.get_param("~min_forward_attempts", 20))
        self.min_turn_attempts = int(rospy.get_param("~min_turn_attempts", 8))
        self.min_planner_reverse_requests = int(
            rospy.get_param("~min_planner_reverse_requests", 10)
        )
        self.turn_attempt_grace = float(rospy.get_param("~turn_attempt_grace", 22.0))

        self.min_progress_distance = float(rospy.get_param("~min_progress_distance", 0.16))
        self.min_progress_yaw = float(rospy.get_param("~min_progress_yaw", 0.45))
        self.goal_progress_enabled = bool(rospy.get_param("~goal_progress_enabled", True))
        self.goal_progress_tolerance = float(rospy.get_param("~goal_progress_tolerance", 0.16))
        self.yaw_only_progress_enabled = bool(rospy.get_param("~yaw_only_progress_enabled", False))
        self.front_obstacle_distance = float(rospy.get_param("~front_obstacle_distance", 0.65))
        self.front_obstacle_wait = float(rospy.get_param("~front_obstacle_wait", 12.0))
        self.front_angle = float(rospy.get_param("~front_angle", 0.65))
        self.tick_period = float(rospy.get_param("~tick_period", 0.10))

        self._publisher = rospy.Publisher(self.teleop_topic, AckermannDriveStamped, queue_size=1)
        self._clear_costmaps = None
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._active_goal_id = None
        self._active_goal_since = 0.0
        self._reverse_count = 0
        self._clear_requested = False
        self._clear_wall = 0.0
        self._cooldown_until = 0.0
        self._last_defer_log = 0.0

        self._pose = None
        self._progress_pose = None
        self._current_goal = None
        self._best_goal_distance = None
        self._last_progress_wall = 0.0
        self._front_min = float("inf")
        self._front_obstacle_since = None
        self._commands = deque(maxlen=300)
        self._raw_commands = deque(maxlen=300)
        self._last_turn_steering = 0.0

        rospy.Subscriber(self.status_topic, GoalStatusArray, self._status_callback, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber(self.nav_command_topic, AckermannDriveStamped, self._command_callback, queue_size=1)
        if self.raw_cmd_vel_topic:
            rospy.Subscriber(self.raw_cmd_vel_topic, Twist, self._raw_command_callback, queue_size=1)
        rospy.loginfo("ackermann reverse recovery publishing short recoveries on %s", self.teleop_topic)

    def _goal_callback(self, message):
        self._current_goal = message
        self._best_goal_distance = None

    def _status_callback(self, message):
        active = self._latest_active_status(message)
        if active is None:
            if self._active_goal_id is not None:
                self._reset_goal_state(None)
            return

        goal_id = active.goal_id.id or "%s.%s" % (
            active.goal_id.stamp.secs,
            active.goal_id.stamp.nsecs,
        )
        if goal_id != self._active_goal_id:
            self._reset_goal_state(goal_id)

    @staticmethod
    def _latest_active_status(message):
        active_states = (GoalStatus.PENDING, GoalStatus.ACTIVE)
        candidates = [status for status in message.status_list if status.status in active_states]
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda status: (
                status.goal_id.stamp.secs,
                status.goal_id.stamp.nsecs,
                status.goal_id.id,
            ),
        )

    def _reset_goal_state(self, goal_id):
        self._active_goal_id = goal_id
        now = time.monotonic()
        self._active_goal_since = now if goal_id is not None else 0.0
        self._reverse_count = 0
        self._clear_requested = False
        self._clear_wall = 0.0
        self._cooldown_until = 0.0
        self._progress_pose = self._pose
        self._last_progress_wall = now
        self._front_obstacle_since = None
        self._commands.clear()
        self._raw_commands.clear()
        self._best_goal_distance = None

    def _amcl_callback(self, message):
        self._pose = _pose_tuple(message.pose.pose)

    def _odom_callback(self, message):
        if self._pose is None:
            self._pose = _pose_tuple(message.pose.pose)

    def _scan_callback(self, message):
        ranges = []
        for index, value in enumerate(message.ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= self.front_angle:
                ranges.append(value)
        self._front_min = min(ranges) if ranges else float("inf")

    def _command_callback(self, message):
        self._commands.append(
            (
                time.monotonic(),
                float(message.drive.speed),
                float(message.drive.steering_angle),
            )
        )
        if message.drive.speed >= 0.0 and abs(message.drive.steering_angle) > 0.07:
            self._last_turn_steering = float(message.drive.steering_angle)

    def _raw_command_callback(self, message):
        self._raw_commands.append(
            (
                time.monotonic(),
                float(message.linear.x),
                float(message.angular.z),
            )
        )

    def _update_progress(self, now):
        if self.goal_progress_enabled and self._current_goal is not None:
            goal_distance = self._goal_distance()
            if goal_distance is not None:
                if (
                    self._best_goal_distance is None
                    or goal_distance <= self._best_goal_distance - self.goal_progress_tolerance
                ):
                    self._best_goal_distance = goal_distance
                    self._last_progress_wall = now
                    self._progress_pose = self._pose
                    self._clear_requested = False
                return

        if self._pose is None:
            return
        if self._progress_pose is None:
            self._progress_pose = self._pose
            self._last_progress_wall = now
            return

        x, y, yaw = self._pose
        px, py, pyaw = self._progress_pose
        moved = math.hypot(x - px, y - py)
        turned = abs(_wrap_to_pi(yaw - pyaw))
        if moved >= self.min_progress_distance or (
            self.yaw_only_progress_enabled and turned >= self.min_progress_yaw
        ):
            self._progress_pose = self._pose
            self._last_progress_wall = now
            self._clear_requested = False

    def _goal_distance(self):
        goal = self._current_goal
        if goal is None or not goal.header.frame_id:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                goal.header.frame_id,
                self.frame_id,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

        robot = transform.transform.translation
        dx = robot.x - goal.pose.position.x
        dy = robot.y - goal.pose.position.y
        if not math.isfinite(dx) or not math.isfinite(dy):
            return None
        return math.hypot(dx, dy)

    def _attempt_counts(self, now):
        forward = 0
        turn = 0
        planner_reverse = 0
        cutoff = now - self.command_window
        for stamp, speed, steering in self._commands:
            if stamp < cutoff:
                continue
            if speed > 0.015:
                forward += 1
            if speed >= 0.0 and abs(steering) > 0.07:
                turn += 1
        for stamp, linear_x, angular_z in self._raw_commands:
            if stamp < cutoff:
                continue
            if linear_x < -0.005:
                planner_reverse += 1
                if abs(angular_z) > 0.07:
                    turn += 1
        return forward, turn, planner_reverse

    def _defer(self, now, reason):
        if now - self._last_defer_log >= 2.0:
            rospy.loginfo("Reverse recovery deferred: %s", reason)
            self._last_defer_log = now

    def _should_reverse(self, now):
        if self._active_goal_id is None:
            return False
        if self._pose is None:
            self._defer(now, "pose unavailable")
            return False
        if self._reverse_count >= self.max_reverses_per_goal:
            return False
        if now < self._cooldown_until:
            return False
        if now - self._active_goal_since < self.min_goal_age:
            return False
        if now - self._last_progress_wall < self.stuck_timeout:
            return False

        forward_attempts, turn_attempts, planner_reverse_requests = self._attempt_counts(now)
        has_planner_reverse_stall = planner_reverse_requests >= self.min_planner_reverse_requests
        if forward_attempts < self.min_forward_attempts and not has_planner_reverse_stall:
            self._defer(now, "waiting for forward attempts before reversing")
            return False

        no_progress_duration = now - self._last_progress_wall
        if (
            turn_attempts < self.min_turn_attempts
            and no_progress_duration < self.stuck_timeout + self.turn_attempt_grace
        ):
            self._defer(now, "turning attempts still have priority over reverse")
            return False

        if not self._clear_requested:
            self._request_costmap_clear(now)
            return False
        if now - self._clear_wall < self.post_clear_wait:
            self._defer(now, "waiting after costmap clear/replan")
            return False

        if self._front_min <= self.front_obstacle_distance:
            if self._front_obstacle_since is None:
                self._front_obstacle_since = now
            if now - self._front_obstacle_since < self.front_obstacle_wait:
                self._defer(now, "waiting for front obstacle before short reverse")
                return False
        else:
            self._front_obstacle_since = None

        return True

    def _request_costmap_clear(self, now):
        if self._clear_costmaps is None:
            self._clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_service, Empty)
        try:
            self._clear_costmaps()
            rospy.logwarn("Navigation appears stuck; cleared costmaps before considering reverse")
        except rospy.ServiceException as exc:
            rospy.logwarn("Could not clear costmaps before reverse recovery: %s", exc)
        self._clear_requested = True
        self._clear_wall = now

    def _command(self, speed, steering=0.0):
        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = self.frame_id
        command.drive.speed = speed
        command.drive.steering_angle = steering
        return command

    def _perform_reverse(self):
        self._reverse_count += 1
        steering = self._reverse_steering()
        rospy.logwarn(
            "Performing short low-speed reverse recovery %d/%d after forward/turn attempts stalled (steering %.3f)",
            self._reverse_count,
            self.max_reverses_per_goal,
            steering,
        )
        end_wall = time.monotonic() + self.reverse_duration
        while not rospy.is_shutdown() and time.monotonic() < end_wall:
            self._publisher.publish(self._command(self.reverse_speed, steering))
            time.sleep(min(0.10, max(0.02, self.tick_period)))

        stop_end = time.monotonic() + self.stop_duration
        while not rospy.is_shutdown() and time.monotonic() < stop_end:
            self._publisher.publish(self._command(0.0))
            time.sleep(min(0.10, max(0.02, self.tick_period)))

        now = time.monotonic()
        self._cooldown_until = now + self.recovery_cooldown
        self._last_progress_wall = now
        self._progress_pose = self._pose
        self._clear_requested = False
        self._front_obstacle_since = None
        self._commands.clear()
        self._raw_commands.clear()

    def _reverse_steering(self):
        if self.reverse_steering_angle <= 0.0:
            return 0.0
        if abs(self._last_turn_steering) <= 0.07:
            return 0.0
        sign = -1.0 if self.reverse_opposes_last_turn else 1.0
        return sign * math.copysign(self.reverse_steering_angle, self._last_turn_steering)

    def run(self):
        while not rospy.is_shutdown():
            now = time.monotonic()
            self._update_progress(now)
            if self._should_reverse(now):
                self._perform_reverse()
            time.sleep(max(0.02, self.tick_period))


def main():
    rospy.init_node("ackermann_reverse_recovery")
    AckermannReverseRecovery().run()


if __name__ == "__main__":
    main()
