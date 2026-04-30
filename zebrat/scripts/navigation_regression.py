#!/usr/bin/env python3

import math
import os
import sys
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion

sys.path.insert(0, os.path.dirname(__file__))
from goal_safety import CostmapGoalResolver, normalize_topic_list  # noqa: E402


STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


class NavigationRegression:
    def __init__(self):
        self.action_name = rospy.get_param("~action_name", "/move_base")
        self.goal_set = rospy.get_param("~goal_set", "regression")
        self.min_safe_scan = float(rospy.get_param("~min_safe_scan", 0.10))
        self.default_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.server_timeout = float(rospy.get_param("~server_timeout", 90.0))
        self.startup_delay = float(rospy.get_param("~startup_delay", 5.0))
        self.continue_on_failure = bool(rospy.get_param("~continue_on_failure", False))
        self.idle_command_timeout = float(rospy.get_param("~idle_command_timeout", 20.0))
        self.command_topic = rospy.get_param("~command_topic", "/ackermann_cmd")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.goals = rospy.get_param(f"~{self.goal_set}_goals", [])
        self.safe_goal_enabled = bool(rospy.get_param("~safe_goal_enabled", True))
        self.safe_goal_accept_xy_on_adjusted = bool(rospy.get_param("~safe_goal_accept_xy_on_adjusted", True))
        self.safe_goal_adjusted_xy_tolerance = float(rospy.get_param("~safe_goal_adjusted_xy_tolerance", 0.25))

        if not self.goals:
            raise rospy.ROSInitException(f"No goals configured for set '{self.goal_set}'")

        self._client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)
        self._goal_resolver = None
        if self.safe_goal_enabled:
            costmap_topics = normalize_topic_list(
                rospy.get_param(
                    "~safe_goal_costmap_topics",
                    ["/move_base/local_costmap/costmap", "/move_base/global_costmap/costmap"],
                )
            )
            self._goal_resolver = CostmapGoalResolver(
                costmap_topics=costmap_topics,
                occupied_threshold=rospy.get_param("~safe_goal_occupied_threshold", 65),
                unknown_is_occupied=rospy.get_param("~safe_goal_unknown_is_occupied", True),
                target_check_radius=rospy.get_param("~safe_goal_target_check_radius", 0.35),
                search_radius=rospy.get_param("~safe_goal_search_radius", 1.2),
                search_step=rospy.get_param("~safe_goal_search_step", 0.10),
                wait_timeout=rospy.get_param("~safe_goal_wait_timeout", 6.0),
                wait_check_period=rospy.get_param("~safe_goal_wait_check_period", 0.5),
                costmap_wait_timeout=rospy.get_param("~safe_goal_costmap_wait_timeout", 10.0),
                use_dynamic_routes=rospy.get_param("~safe_goal_use_dynamic_routes", True),
                dynamic_route_inflation=rospy.get_param("~safe_goal_dynamic_route_inflation", 0.42),
            )
        self._last_scan_min = float("inf")
        self._last_command_wall = time.monotonic()
        self._collision_proxy_triggered = False
        self._odom_pose = None
        self._map_pose = None
        self._tf_listener = tf.TransformListener()

        rospy.Subscriber("/scan", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber(self.command_topic, AckermannDriveStamped, self._command_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_callback, queue_size=1)

    def _scan_callback(self, message):
        finite_ranges = [value for value in message.ranges if math.isfinite(value) and value > 0.0]
        if not finite_ranges:
            return
        self._last_scan_min = min(finite_ranges)
        if self._last_scan_min < self.min_safe_scan:
            self._collision_proxy_triggered = True

    def _command_callback(self, message):
        if abs(message.drive.speed) + abs(message.drive.steering_angle) > 1e-3:
            self._last_command_wall = time.monotonic()

    def _odom_callback(self, message):
        self._odom_pose = self._extract_pose_tuple(message.pose.pose)

    def _amcl_pose_callback(self, message):
        self._map_pose = self._extract_pose_tuple(message.pose.pose)

    @staticmethod
    def _extract_pose_tuple(pose):
        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(yaw),
        )

    def wait_for_server(self):
        required_topics = {
            f"{self.action_name}/status",
            f"{self.action_name}/feedback",
            f"{self.action_name}/result",
        }
        deadline = time.monotonic() + self.server_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            published_topics = {topic for topic, _msg_type in rospy.get_published_topics()}
            if required_topics.issubset(published_topics) and self._client.wait_for_server(rospy.Duration(1.0)):
                return
            rospy.loginfo_throttle(5.0, "Waiting for action server %s", self.action_name)
            time.sleep(0.5)

        raise rospy.ROSInitException(f"Timed out waiting for action server {self.action_name}")

    def _to_pose(self, item):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(item["x"])
        pose.pose.position.y = float(item["y"])
        yaw = float(item.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _to_goal(self, item):
        goal = MoveBaseGoal()
        goal.target_pose = self._to_pose(item)
        return goal

    def _resolve_goal_item(self, goal_item, name):
        if not self._goal_resolver or not goal_item.get("safe_goal", True):
            return dict(goal_item)

        requested_pose = self._to_pose(goal_item)
        resolved = self._goal_resolver.resolve_pose(requested_pose, name)
        resolved_item = dict(goal_item)
        if resolved.blocked:
            resolved_item["safe_goal_blocked"] = True
            resolved_item["safe_goal_blocked_reason"] = resolved.reason
            rospy.logerr(
                "Navigation goal %s is unsafe and has no replacement; not sending requested pose",
                name,
            )
            return resolved_item

        if resolved.adjusted:
            resolved_item["requested_x"] = float(goal_item["x"])
            resolved_item["requested_y"] = float(goal_item["y"])
            resolved_item["safe_goal_adjusted"] = True
            resolved_item["x"] = float(resolved.pose.pose.position.x)
            resolved_item["y"] = float(resolved.pose.pose.position.y)
            rospy.loginfo(
                "Using adjusted navigation goal %s: requested=(%.2f, %.2f), sent=(%.2f, %.2f)",
                name,
                resolved_item["requested_x"],
                resolved_item["requested_y"],
                resolved_item["x"],
                resolved_item["y"],
            )
        return resolved_item

    def _is_allowed_terminal_state(self, status, goal_item):
        if status == GoalStatus.SUCCEEDED:
            return True
        if goal_item.get("allow_blocked", False) and status in (GoalStatus.ABORTED, GoalStatus.PREEMPTED):
            return True
        return False

    def _should_stop_after_accept(self, status, goal_item):
        if goal_item.get("stop_after_accept", False):
            return True
        if (
            goal_item.get("stop_on_blocked", False)
            and status in (GoalStatus.ABORTED, GoalStatus.PREEMPTED)
        ):
            return True
        return False

    def _wait_for_result(self, goal_item):
        deadline = time.monotonic() + float(goal_item.get("timeout", self.default_timeout))
        name = goal_item.get("name", f"{goal_item['x']},{goal_item['y']}")
        while not rospy.is_shutdown():
            state = self._client.get_state()
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.PREEMPTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                return state
            if self._collision_proxy_triggered:
                self._client.cancel_goal()
                rospy.logerr("Collision proxy triggered while driving to %s (min scan %.3f)", name, self._last_scan_min)
                return GoalStatus.ABORTED
            if self._adjusted_goal_xy_reached(goal_item):
                self._client.cancel_goal()
                rospy.loginfo(
                    "Adjusted goal %s reached XY tolerance %.2fm; accepting without final yaw alignment",
                    name,
                    self.safe_goal_adjusted_xy_tolerance,
                )
                return GoalStatus.SUCCEEDED
            if time.monotonic() > deadline:
                self._client.cancel_goal()
                rospy.logwarn("Goal %s timed out after %.1fs", name, float(goal_item.get("timeout", self.default_timeout)))
                return GoalStatus.ABORTED
            if time.monotonic() - self._last_command_wall > self.idle_command_timeout:
                rospy.logwarn("Goal %s has been idle for %.1fs", name, self.idle_command_timeout)
                self._last_command_wall = time.monotonic()
            time.sleep(0.2)

        self._client.cancel_all_goals()
        return GoalStatus.PREEMPTED

    def _adjusted_goal_xy_reached(self, goal_item):
        if not self.safe_goal_accept_xy_on_adjusted or not goal_item.get("safe_goal_adjusted", False):
            return False
        pose = self._current_map_pose() or self._map_pose or self._odom_pose
        if pose is None:
            return False
        x, y, _yaw = pose
        return math.hypot(float(goal_item["x"]) - x, float(goal_item["y"]) - y) <= self.safe_goal_adjusted_xy_tolerance

    def _current_map_pose(self):
        try:
            translation, rotation = self._tf_listener.lookupTransform(
                self.frame_id,
                "base_footprint",
                rospy.Time(0),
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        _, _, yaw = euler_from_quaternion(rotation)
        return float(translation[0]), float(translation[1]), float(yaw)

    @staticmethod
    def _format_elapsed(elapsed_seconds):
        return f"{elapsed_seconds:.1f}s"

    @staticmethod
    def _wrap_to_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _format_goal_metrics(self, goal_item):
        pose = self._current_map_pose()
        pose_label = "tf_pose"
        if pose is None:
            pose = self._map_pose
            pose_label = "map_pose"
        if pose is None:
            pose = self._odom_pose
            pose_label = "odom_pose"
        if pose is None:
            return "pose unavailable"

        x, y, yaw = pose
        goal_x = float(goal_item["x"])
        goal_y = float(goal_item["y"])
        goal_yaw = float(goal_item.get("yaw", 0.0))
        position_error = math.hypot(goal_x - x, goal_y - y)
        yaw_error = abs(self._wrap_to_pi(goal_yaw - yaw))
        metrics = (
            f"{pose_label}=({x:.2f},{y:.2f},{yaw:.2f}) "
            f"goal_err=({position_error:.2f}m,{yaw_error:.2f}rad)"
        )
        if "requested_x" in goal_item and "requested_y" in goal_item:
            requested_error = math.hypot(float(goal_item["requested_x"]) - x, float(goal_item["requested_y"]) - y)
            metrics += f" requested_goal_err={requested_error:.2f}m"
        return metrics

    def run(self):
        if self.startup_delay > 0.0:
            rospy.loginfo("Delaying navigation regression start by %.1fs", self.startup_delay)
            time.sleep(self.startup_delay)
        self.wait_for_server()
        failures = []

        for index, goal_item in enumerate(self.goals, start=1):
            name = goal_item.get("name", f"goal_{index}")
            nav_goal_item = self._resolve_goal_item(goal_item, name)
            self._collision_proxy_triggered = False
            goal_wall_start = time.monotonic()
            if nav_goal_item.get("safe_goal_blocked", False):
                status = GoalStatus.ABORTED
                status_name = STATUS_NAMES.get(status, str(status))
                elapsed = self._format_elapsed(time.monotonic() - goal_wall_start)
                reason = nav_goal_item.get("safe_goal_blocked_reason", "blocked")
                if self._is_allowed_terminal_state(status, goal_item):
                    rospy.loginfo(
                        "[%d/%d] Goal %s blocked before send with %s after %s (%s)",
                        index,
                        len(self.goals),
                        name,
                        status_name,
                        elapsed,
                        reason,
                    )
                    if self._should_stop_after_accept(status, goal_item):
                        rospy.loginfo("Stopping regression after blocked terminal goal %s", name)
                        break
                    continue

                failures.append(f"{name}: BLOCKED({reason})")
                rospy.logerr(
                    "[%d/%d] Goal %s rejected before send: %s",
                    index,
                    len(self.goals),
                    name,
                    reason,
                )
                if not self.continue_on_failure:
                    break
                continue

            self._client.send_goal(self._to_goal(nav_goal_item))
            rospy.loginfo(
                "[%d/%d] Sent goal %s to (%.2f, %.2f, %.2f)",
                index,
                len(self.goals),
                name,
                float(nav_goal_item["x"]),
                float(nav_goal_item["y"]),
                float(nav_goal_item.get("yaw", 0.0)),
            )
            status = self._wait_for_result(nav_goal_item)
            status_name = STATUS_NAMES.get(status, str(status))
            elapsed = self._format_elapsed(time.monotonic() - goal_wall_start)
            metrics = self._format_goal_metrics(nav_goal_item)
            if self._is_allowed_terminal_state(status, goal_item):
                rospy.loginfo(
                    "[%d/%d] Goal %s finished with %s after %s (%s)",
                    index,
                    len(self.goals),
                    name,
                    status_name,
                    elapsed,
                    metrics,
                )
                if self._should_stop_after_accept(status, goal_item):
                    rospy.loginfo("Stopping regression after terminal goal %s", name)
                    break
                continue

            failures.append(f"{name}: {status_name}")
            rospy.logerr(
                "[%d/%d] Goal %s failed with %s after %s (%s)",
                index,
                len(self.goals),
                name,
                status_name,
                elapsed,
                metrics,
            )
            if not self.continue_on_failure:
                break

        if failures:
            raise SystemExit("Navigation regression failed: " + "; ".join(failures))

        rospy.loginfo("Navigation regression completed successfully for goal set '%s'", self.goal_set)


def main():
    rospy.init_node("navigation_regression")
    regression = NavigationRegression()
    regression.run()


if __name__ == "__main__":
    try:
        main()
    except SystemExit as exc:
        if str(exc):
            rospy.logerr(str(exc))
        sys.exit(1)
