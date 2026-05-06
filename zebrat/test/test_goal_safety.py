#!/usr/bin/env python3

import copy
import os
import sys
import time
import unittest

from costmap_converter.msg import ObstacleArrayMsg
from costmap_converter.msg import ObstacleMsg
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from goal_safety import CostmapGoalResolver  # noqa: E402
from goal_safety import ResolvedGoal  # noqa: E402
from safe_goal_relay import SafeGoalRelay  # noqa: E402


def _pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def _obstacle(obstacle_id, x, y, radius, vx, vy):
    obstacle = ObstacleMsg()
    obstacle.id = obstacle_id
    obstacle.radius = radius
    point = Point32()
    point.x = x
    point.y = y
    obstacle.polygon.points = [point]
    obstacle.velocities.twist.linear.x = vx
    obstacle.velocities.twist.linear.y = vy
    return obstacle


class DynamicPathSafetyTest(unittest.TestCase):
    def _resolver(self, start, obstacle):
        message = ObstacleArrayMsg()
        message.header.frame_id = "map"
        message.obstacles.append(obstacle)

        resolver = CostmapGoalResolver.__new__(CostmapGoalResolver)
        resolver.use_dynamic_obstacle_predictions = True
        resolver.dynamic_obstacle_prediction_timeout = 1.0
        resolver.dynamic_obstacle_prediction_horizon = 1.0
        resolver.dynamic_obstacle_path_inflation = 0.42
        resolver._dynamic_obstacles = {"test": (message, time.monotonic())}
        resolver._transform_pose = lambda pose, _frame_id: pose
        resolver._current_pose = lambda _frame_id: start
        return resolver

    def test_path_crossing_predicted_obstacle_is_blocked(self):
        resolver = self._resolver(
            _pose(1.95, -1.65),
            _obstacle(1, 2.75, -1.50, 0.24, 0.0, -0.55),
        )

        self.assertTrue(resolver._path_to_pose_blocked_by_dynamic_obstacles(_pose(3.45, -1.65)))

    def test_path_clear_of_predicted_obstacle_is_allowed(self):
        resolver = self._resolver(
            _pose(1.95, -2.80),
            _obstacle(1, 2.75, -1.50, 0.24, 0.0, -0.55),
        )

        self.assertFalse(resolver._path_to_pose_blocked_by_dynamic_obstacles(_pose(3.45, -2.80)))


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeResolver:
    def __init__(self, results):
        self.results = list(results)
        self.calls = []

    def resolve_pose(self, pose, name="goal", wait=True, log_blocked=True):
        self.calls.append(
            {
                "pose": copy.deepcopy(pose),
                "name": name,
                "wait": wait,
                "log_blocked": log_blocked,
            }
        )
        return self.results.pop(0)


class SafeGoalRelayPendingRetryTest(unittest.TestCase):
    def _relay(self, results):
        relay = SafeGoalRelay.__new__(SafeGoalRelay)
        relay.accept_xy_on_adjusted = False
        relay.retry_rejected_goals = True
        relay.retry_timeout = 20.0
        relay.retry_period = 1.0
        relay.resolver = _FakeResolver(results)
        relay.publisher = _FakePublisher()
        relay.cancel_publisher = _FakePublisher()
        relay._active_adjusted_goal = None
        relay._pending_goal = None
        relay._pending_goal_created_wall = 0.0
        relay._pending_goal_deadline = 0.0
        relay._pending_goal_retry_count = 0
        relay._pending_goal_reason = ""
        return relay

    def test_blocked_goal_is_kept_pending_and_retried_without_waiting(self):
        requested = _pose(2.0, 1.0)
        relay = self._relay(
            [
                ResolvedGoal(requested, reason="no_replacement", blocked=True),
                ResolvedGoal(requested),
            ]
        )

        relay._goal_callback(requested)
        self.assertEqual(len(relay.cancel_publisher.messages), 1)
        self.assertEqual(len(relay.publisher.messages), 0)
        self.assertIsNotNone(relay._pending_goal)
        self.assertTrue(relay.resolver.calls[0]["wait"])

        relay._retry_timer_callback(None)
        self.assertEqual(len(relay.publisher.messages), 1)
        self.assertIsNone(relay._pending_goal)
        self.assertFalse(relay.resolver.calls[1]["wait"])
        self.assertFalse(relay.resolver.calls[1]["log_blocked"])

    def test_pending_goal_expires_without_republishing(self):
        requested = _pose(2.0, 1.0)
        relay = self._relay([ResolvedGoal(requested)])
        relay._pending_goal = requested
        relay._pending_goal_created_wall = time.monotonic() - 30.0
        relay._pending_goal_deadline = time.monotonic() - 1.0

        relay._retry_timer_callback(None)

        self.assertEqual(len(relay.publisher.messages), 0)
        self.assertIsNone(relay._pending_goal)
        self.assertEqual(len(relay.resolver.calls), 0)


if __name__ == "__main__":
    unittest.main()
