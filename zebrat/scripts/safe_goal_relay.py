#!/usr/bin/env python3

import os
import sys

import rospy
import tf
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
import math

sys.path.insert(0, os.path.dirname(__file__))
from goal_safety import CostmapGoalResolver, normalize_topic_list  # noqa: E402


class SafeGoalRelay:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/safe_move_base_simple/goal")
        self.output_topic = rospy.get_param("~output_topic", "/move_base_simple/goal")
        self.accept_xy_on_adjusted = bool(rospy.get_param("~accept_xy_on_adjusted", True))
        self.adjusted_xy_tolerance = float(rospy.get_param("~adjusted_xy_tolerance", 0.25))
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
            use_dynamic_routes=rospy.get_param("~use_dynamic_routes", True),
            dynamic_route_inflation=rospy.get_param("~dynamic_route_inflation", 0.42),
        )
        self.publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.cancel_publisher = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self._active_adjusted_goal = None
        self.subscriber = rospy.Subscriber(self.input_topic, PoseStamped, self._goal_callback, queue_size=1)
        self._reach_timer = rospy.Timer(rospy.Duration(0.1), self._reach_timer_callback)
        rospy.loginfo("Safe goal relay listening on %s and publishing to %s", self.input_topic, self.output_topic)

    def _goal_callback(self, message):
        resolved = self.resolver.resolve_pose(message, "simple goal")
        output = resolved.pose
        output.header.stamp = rospy.Time.now()
        self._active_adjusted_goal = output if resolved.adjusted and self.accept_xy_on_adjusted else None
        self.publisher.publish(output)

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
