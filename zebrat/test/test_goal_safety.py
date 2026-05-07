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
from nav_msgs.msg import OccupancyGrid


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


def _grid(width, height, resolution, data):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.orientation.w = 1.0
    grid.data = list(data)
    return grid


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


class UnknownFallbackGoalSafetyTest(unittest.TestCase):
    def _resolver(self, grid, allow_unknown_fallback=True, use_dynamic_on_fallback=False):
        resolver = CostmapGoalResolver.__new__(CostmapGoalResolver)
        resolver.occupied_threshold = 65
        resolver.unknown_is_occupied = True
        resolver.target_check_radius = 0.0
        resolver.search_radius = 0.0
        resolver.search_step = 0.10
        resolver.wait_timeout = 0.0
        resolver.wait_check_period = 0.5
        resolver.costmap_wait_timeout = 0.0
        resolver.use_dynamic_routes = False
        resolver.dynamic_route_inflation = 0.42
        resolver.use_dynamic_obstacle_predictions = True
        resolver.dynamic_obstacle_path_inflation = 0.42
        resolver.dynamic_obstacle_prediction_timeout = 1.0
        resolver.dynamic_obstacle_prediction_horizon = 1.0
        resolver.allow_unknown_fallback = allow_unknown_fallback
        resolver.unknown_fallback_use_dynamic_obstacle_predictions = use_dynamic_on_fallback
        resolver.base_frame = "base_footprint"
        resolver._costmaps = {"test": grid}
        resolver._dynamic_obstacles = {}
        resolver._dynamic_route_segments = []
        resolver.wait_for_costmaps = lambda: True
        resolver._transform_pose = lambda pose, _frame_id: pose
        resolver._current_pose = lambda _frame_id: _pose(0.0, 0.0)
        return resolver

    def test_unknown_goal_is_allowed_only_as_fallback(self):
        resolver = self._resolver(_grid(3, 3, 1.0, [-1] * 9))

        resolved = resolver.resolve_pose(_pose(1.0, 1.0), "unknown goal", wait=False, log_blocked=False)

        self.assertFalse(resolved.blocked)
        self.assertEqual(resolved.reason, "unknown_fallback")

    def test_occupied_goal_is_not_allowed_by_unknown_fallback(self):
        data = [-1] * 9
        data[4] = 100
        resolver = self._resolver(_grid(3, 3, 1.0, data))

        resolved = resolver.resolve_pose(_pose(1.0, 1.0), "occupied goal", wait=False, log_blocked=False)

        self.assertTrue(resolved.blocked)
        self.assertEqual(resolved.reason, "no_replacement")

    def test_unknown_fallback_can_bypass_conservative_dynamic_prediction(self):
        message = ObstacleArrayMsg()
        message.header.frame_id = "map"
        message.obstacles.append(_obstacle(18, 0.5, 0.0, 0.30, 1.0, 0.0))
        resolver = self._resolver(_grid(3, 3, 1.0, [-1] * 9))
        resolver._dynamic_obstacles = {"test": (message, time.monotonic())}

        resolved = resolver.resolve_pose(_pose(2.0, 0.0), "unknown dynamic goal", wait=False, log_blocked=False)

        self.assertFalse(resolved.blocked)
        self.assertEqual(resolved.reason, "unknown_fallback")


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
