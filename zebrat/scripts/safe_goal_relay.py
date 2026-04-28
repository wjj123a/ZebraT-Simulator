#!/usr/bin/env python3

import os
import sys

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

sys.path.insert(0, os.path.dirname(__file__))
from goal_safety import CostmapGoalResolver, normalize_topic_list  # noqa: E402


class SafeGoalRelay:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/safe_move_base_simple/goal")
        self.output_topic = rospy.get_param("~output_topic", "/move_base_simple/goal")
        self.accept_xy_on_adjusted = bool(rospy.get_param("~accept_xy_on_adjusted", True))
        self.adjusted_xy_tolerance = float(rospy.get_param("~adjusted_xy_tolerance", 0.25))
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
        self.pose_subscriber = rospy.Subscriber(
            rospy.get_param("~pose_topic", "/amcl_pose"),
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=1,
        )
        rospy.loginfo("Safe goal relay listening on %s and publishing to %s", self.input_topic, self.output_topic)

    def _goal_callback(self, message):
        resolved = self.resolver.resolve_pose(message, "simple goal")
        output = resolved.pose
        output.header.stamp = rospy.Time.now()
        self._active_adjusted_goal = output if resolved.adjusted and self.accept_xy_on_adjusted else None
        self.publisher.publish(output)

    def _pose_callback(self, message):
        if self._active_adjusted_goal is None:
            return
        dx = self._active_adjusted_goal.pose.position.x - message.pose.pose.position.x
        dy = self._active_adjusted_goal.pose.position.y - message.pose.pose.position.y
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
