#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from dynamic_obstacle_predictions import build_obstacle_message  # noqa: E402


class DynamicObstaclePredictionMessageTest(unittest.TestCase):
    def test_pedestrian_uses_point_with_radius_and_velocity(self):
        message = build_obstacle_message(
            3,
            {"name": "pedestrian", "type": "pedestrian", "radius": 0.24},
            1.25,
            -0.50,
            math.pi / 2.0,
            0.0,
            0.55,
        )

        self.assertEqual(message.id, 3)
        self.assertAlmostEqual(message.radius, 0.24)
        self.assertEqual(len(message.polygon.points), 1)
        self.assertAlmostEqual(message.polygon.points[0].x, 1.25)
        self.assertAlmostEqual(message.polygon.points[0].y, -0.50)
        self.assertAlmostEqual(message.velocities.twist.linear.x, 0.0)
        self.assertAlmostEqual(message.velocities.twist.linear.y, 0.55)

    def test_vehicle_uses_rotated_rectangle_polygon(self):
        message = build_obstacle_message(
            4,
            {"name": "cart", "type": "cart", "size": [0.80, 0.40, 0.50]},
            2.0,
            1.0,
            0.0,
            0.35,
            0.0,
        )

        self.assertEqual(message.id, 4)
        self.assertAlmostEqual(message.radius, 0.0)
        self.assertEqual(len(message.polygon.points), 4)
        self.assertAlmostEqual(message.polygon.points[0].x, 2.40)
        self.assertAlmostEqual(message.polygon.points[0].y, 1.20)
        self.assertAlmostEqual(message.polygon.points[2].x, 1.60)
        self.assertAlmostEqual(message.polygon.points[2].y, 0.80)
        self.assertAlmostEqual(message.velocities.twist.linear.x, 0.35)


if __name__ == "__main__":
    unittest.main()
