#!/usr/bin/env python3

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


if __name__ == "__main__":
    unittest.main()
