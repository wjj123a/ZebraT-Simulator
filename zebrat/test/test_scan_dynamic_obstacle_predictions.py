#!/usr/bin/env python3

import math
import os
import sys
import unittest

from sensor_msgs.msg import LaserScan


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from scan_dynamic_obstacle_predictions import (  # noqa: E402
    SensorDetection,
    TrackedSensorObstacle,
    build_sensor_obstacle_message,
    cluster_scan_points,
    cluster_to_detection,
)


def _scan(ranges, angle_min=-0.10, angle_increment=0.05):
    scan = LaserScan()
    scan.header.frame_id = "laser_link"
    scan.angle_min = angle_min
    scan.angle_increment = angle_increment
    scan.range_min = 0.12
    scan.range_max = 8.0
    scan.ranges = list(ranges)
    return scan


class ScanDynamicObstaclePredictionTest(unittest.TestCase):
    def test_scan_points_are_clustered_across_finite_runs(self):
        scan = _scan(
            [
                float("inf"),
                1.00,
                1.01,
                1.00,
                float("inf"),
                2.00,
                2.02,
                2.00,
            ]
        )

        clusters = cluster_scan_points(scan, cluster_max_gap=0.16, min_points=3)

        self.assertEqual(len(clusters), 2)
        self.assertEqual([len(cluster) for cluster in clusters], [3, 3])

    def test_large_scan_cluster_is_not_reported_as_dynamic_obstacle(self):
        detection = cluster_to_detection(
            [(0.0, 0.0), (0.9, 0.0), (1.8, 0.0)],
            max_cluster_diameter=0.8,
        )

        self.assertIsNone(detection)

    def test_detection_builds_point_obstacle_with_radius_and_velocity(self):
        message = build_sensor_obstacle_message(7, 1.25, -0.50, 0.24, 0.0, 0.55)

        self.assertEqual(message.id, 7)
        self.assertAlmostEqual(message.radius, 0.24)
        self.assertEqual(len(message.polygon.points), 1)
        self.assertAlmostEqual(message.polygon.points[0].x, 1.25)
        self.assertAlmostEqual(message.polygon.points[0].y, -0.50)
        self.assertAlmostEqual(message.velocities.twist.linear.x, 0.0)
        self.assertAlmostEqual(message.velocities.twist.linear.y, 0.55)

    def test_track_estimates_and_clamps_obstacle_velocity(self):
        track = TrackedSensorObstacle(
            3,
            SensorDetection(0.0, 0.0, 0.20, 5),
            wall_time=10.0,
            velocity_alpha=1.0,
            max_speed=0.5,
        )

        track.update(SensorDetection(3.0, 4.0, 0.18, 5), wall_time=11.0)

        self.assertAlmostEqual(math.hypot(track.vx, track.vy), 0.5)
        self.assertAlmostEqual(track.vx, 0.3)
        self.assertAlmostEqual(track.vy, 0.4)
        self.assertAlmostEqual(track.radius, 0.20)

    def test_stale_track_message_extrapolates_position_and_uncertainty(self):
        track = TrackedSensorObstacle(
            5,
            SensorDetection(1.0, 2.0, 0.25, 5),
            wall_time=10.0,
            velocity_alpha=1.0,
            max_speed=1.0,
        )
        track.update(SensorDetection(1.2, 2.0, 0.25, 5), wall_time=10.5)

        message = track.to_message(wall_time=11.0, radius_growth_rate=0.10)

        self.assertAlmostEqual(message.polygon.points[0].x, 1.4)
        self.assertAlmostEqual(message.polygon.points[0].y, 2.0)
        self.assertAlmostEqual(message.radius, 0.30)

    def test_track_is_marked_moving_only_after_repeated_motion(self):
        track = TrackedSensorObstacle(
            8,
            SensorDetection(0.0, 0.0, 0.25, 5),
            wall_time=10.0,
            velocity_alpha=1.0,
            max_speed=1.0,
        )

        track.update(SensorDetection(0.2, 0.0, 0.25, 5), wall_time=11.0)
        self.assertFalse(track.is_moving(11.0))

        track.update(SensorDetection(0.4, 0.0, 0.25, 5), wall_time=12.0)
        self.assertTrue(track.is_moving(12.0))
        self.assertTrue(track.is_moving(13.9))
        self.assertFalse(track.is_moving(14.1))


if __name__ == "__main__":
    unittest.main()
