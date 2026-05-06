#!/usr/bin/env python3

import copy
import math
import os
import sys
import time

import rospy
import tf
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(__file__))
from goal_safety import CostmapGoalResolver, normalize_topic_list  # noqa: E402


class SafeGoalRelay:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/safe_move_base_simple/goal")
        self.output_topic = rospy.get_param("~output_topic", "/move_base_simple/goal")
        self.accept_xy_on_adjusted = bool(rospy.get_param("~accept_xy_on_adjusted", True))
        self.adjusted_xy_tolerance = float(rospy.get_param("~adjusted_xy_tolerance", 0.25))
        self.retry_rejected_goals = bool(rospy.get_param("~retry_rejected_goals", True))
        self.retry_timeout = max(0.0, float(rospy.get_param("~retry_timeout", 20.0)))
        self.retry_period = max(0.1, float(rospy.get_param("~retry_period", 1.0)))
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self._tf = tf.TransformListener()
        costmap_topics = normalize_topic_list(
            rospy.get_param(
                "~costmap_topics",
                ["/move_base/local_costmap/costmap", "/move_base/global_costmap/costmap"],
            )
        )
        self.resolver = CostmapGoalResolver(
            costmap_topics=costmap_topics,
            occupied_threshold=rospy.get_param("~occupied_threshold", 65),
            unknown_is_occupied=rospy.get_param("~unknown_is_occupied", True),
            target_check_radius=rospy.get_param("~target_check_radius", 0.35),
            search_radius=rospy.get_param("~search_radius", 1.2),
            search_step=rospy.get_param("~search_step", 0.10),
            wait_timeout=rospy.get_param("~wait_timeout", 6.0),
            wait_check_period=rospy.get_param("~wait_check_period", 0.5),
            costmap_wait_timeout=rospy.get_param("~costmap_wait_timeout", 10.0),
            use_dynamic_routes=rospy.get_param("~use_dynamic_routes", False),
            dynamic_route_inflation=rospy.get_param("~dynamic_route_inflation", 0.42),
            use_dynamic_obstacle_predictions=rospy.get_param("~use_dynamic_obstacle_predictions", True),
            dynamic_obstacle_topics=rospy.get_param("~dynamic_obstacle_topics", "/move_base/TebLocalPlannerROS/obstacles"),
            dynamic_obstacle_path_inflation=rospy.get_param("~dynamic_obstacle_path_inflation", 0.42),
            dynamic_obstacle_prediction_timeout=rospy.get_param("~dynamic_obstacle_prediction_timeout", 1.0),
            dynamic_obstacle_prediction_horizon=rospy.get_param("~dynamic_obstacle_prediction_horizon", 1.5),
            base_frame=self.base_frame,
        )
        self.publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.cancel_publisher = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self._active_adjusted_goal = None
        self._pending_goal = None
        self._pending_goal_created_wall = 0.0
        self._pending_goal_deadline = 0.0
        self._pending_goal_retry_count = 0
        self._pending_goal_reason = ""
        self.subscriber = rospy.Subscriber(self.input_topic, PoseStamped, self._goal_callback, queue_size=1)
        self._reach_timer = rospy.Timer(rospy.Duration(0.1), self._reach_timer_callback)
        self._retry_timer = rospy.Timer(rospy.Duration(self.retry_period), self._retry_timer_callback)
        rospy.loginfo("Safe goal relay listening on %s and publishing to %s", self.input_topic, self.output_topic)

    def _goal_callback(self, message):
        self._clear_pending_goal()
        resolved = self.resolver.resolve_pose(message, "simple goal")
        if resolved.blocked:
            self._active_adjusted_goal = None
            self.cancel_publisher.publish(GoalID())
            rospy.logerr(
                "Rejected unsafe simple goal at (%.2f, %.2f): %s",
                message.pose.position.x,
                message.pose.position.y,
                resolved.reason or "blocked",
            )
            self._store_pending_goal(message, resolved.reason or "blocked")
            return

        self._publish_resolved_goal(resolved)

    def _publish_resolved_goal(self, resolved):
        output = copy.deepcopy(resolved.pose)
        output.header.stamp = self._now_stamp()
        self._active_adjusted_goal = output if resolved.adjusted and self.accept_xy_on_adjusted else None
        self.publisher.publish(output)

    @staticmethod
    def _now_stamp():
        return rospy.Time.now() if rospy.core.is_initialized() else rospy.Time(0)

    def _clear_pending_goal(self):
        self._pending_goal = None
        self._pending_goal_created_wall = 0.0
        self._pending_goal_deadline = 0.0
        self._pending_goal_retry_count = 0
        self._pending_goal_reason = ""

    def _store_pending_goal(self, message, reason):
        if not self.retry_rejected_goals or self.retry_timeout <= 0.0:
            return

        now = time.monotonic()
        self._pending_goal = copy.deepcopy(message)
        self._pending_goal_created_wall = now
        self._pending_goal_deadline = now + self.retry_timeout
        self._pending_goal_retry_count = 0
        self._pending_goal_reason = reason
        rospy.logwarn(
            "Keeping rejected simple goal at (%.2f, %.2f) pending for %.1fs; it will be retried every %.1fs",
            message.pose.position.x,
            message.pose.position.y,
            self.retry_timeout,
            self.retry_period,
        )

    def _retry_timer_callback(self, _event):
        if self._pending_goal is None:
            return

        now = time.monotonic()
        if now >= self._pending_goal_deadline:
            rospy.logerr(
                "Pending simple goal at (%.2f, %.2f) expired after %.1fs; waiting for a new RViz goal",
                self._pending_goal.pose.position.x,
                self._pending_goal.pose.position.y,
                now - self._pending_goal_created_wall,
            )
            self._clear_pending_goal()
            return

        self._pending_goal_retry_count += 1
        resolved = self.resolver.resolve_pose(
            self._pending_goal,
            "pending simple goal",
            wait=False,
            log_blocked=False,
        )
        if resolved.blocked:
            rospy.logwarn_throttle(
                2.0,
                "Pending simple goal is still unsafe after %d retries; %.1fs remain",
                self._pending_goal_retry_count,
                max(0.0, self._pending_goal_deadline - now),
            )
            return

        rospy.loginfo(
            "Pending simple goal became safe after %.1fs and %d retries; publishing to move_base",
            now - self._pending_goal_created_wall,
            self._pending_goal_retry_count,
        )
        self._publish_resolved_goal(resolved)
        self._clear_pending_goal()

    def _reach_timer_callback(self, _event):
        if self._active_adjusted_goal is None:
            return

        goal = PoseStamped()
        goal.header.frame_id = self._active_adjusted_goal.header.frame_id
        goal.header.stamp = rospy.Time(0)
        goal.pose = self._active_adjusted_goal.pose
        try:
            self._tf.waitForTransform(self.base_frame, goal.header.frame_id, rospy.Time(0), rospy.Duration(0.05))
            relative_goal = self._tf.transformPose(self.base_frame, goal)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Could not check adjusted goal reach distance in %s: %s",
                self.base_frame,
                exc,
            )
            return

        dx = relative_goal.pose.position.x
        dy = relative_goal.pose.position.y
        if math.hypot(dx, dy) > self.adjusted_xy_tolerance:
            return
        self.cancel_publisher.publish(GoalID())
        rospy.loginfo(
            "Adjusted simple goal reached XY tolerance %.2fm; canceled move_base yaw alignment",
            self.adjusted_xy_tolerance,
        )
        self._active_adjusted_goal = None


def main():
    rospy.init_node("safe_goal_relay")
    SafeGoalRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
