#!/usr/bin/env python3

import os
import sys
import time
import unittest
from collections import deque

from ackermann_msgs.msg import AckermannDriveStamped
from costmap_converter.msg import ObstacleArrayMsg
from costmap_converter.msg import ObstacleMsg
from geometry_msgs.msg import Point32


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from r1_ackermann_controller import R1AckermannController  # noqa: E402
from ackermann_cmd_safety_supervisor import AckermannCmdSafetySupervisor  # noqa: E402
from ackermann_cmd_safety_supervisor import DirectionalScanState  # noqa: E402
from ackermann_reverse_recovery import AckermannReverseRecovery  # noqa: E402
from twist_to_ackermann import TwistToAckermann  # noqa: E402


class AckermannGeometryTest(unittest.TestCase):
    def _controller(self):
        controller = R1AckermannController.__new__(R1AckermannController)
        controller.wheelbase = 0.41893
        controller.front_track = 0.26462
        controller.rear_track = 0.24182
        controller.wheel_radius = 0.04075
        controller.max_speed = 0.22
        controller.max_reverse_speed = 0.08
        controller.max_steering_angle = 0.45
        controller.steering_trim = 0.0
        controller.hold_steering_on_timeout = False
        controller.left_wheel_sign = 1.0
        controller.right_wheel_sign = -1.0
        controller.left_wheel_speed_scale = 1.0
        controller.right_wheel_speed_scale = 1.0
        controller.left_steering_sign = 1.0
        controller.right_steering_sign = 1.0
        return controller

    @staticmethod
    def _command(speed, steering):
        command = AckermannDriveStamped()
        command.drive.speed = speed
        command.drive.steering_angle = steering
        return command

    def test_inside_front_wheel_steers_more_on_left_turn(self):
        left, right, _left_front, _right_front, _left_rear, _right_rear = self._controller()._targets_from_command(
            self._command(0.12, 0.35),
            fresh=True,
        )
        self.assertGreater(abs(left), abs(right))

    def test_inside_front_wheel_steers_more_on_right_turn(self):
        left, right, _left_front, _right_front, _left_rear, _right_rear = self._controller()._targets_from_command(
            self._command(0.12, -0.35),
            fresh=True,
        )
        self.assertGreater(abs(right), abs(left))

    def test_reverse_speed_is_limited_and_wheel_signs_flip(self):
        _left, _right, left_front, right_front, left_rear, right_rear = self._controller()._targets_from_command(
            self._command(-0.20, 0.0),
            fresh=True,
        )
        self.assertAlmostEqual(left_front, -0.08 / 0.04075)
        self.assertAlmostEqual(right_front, 0.08 / 0.04075)
        self.assertAlmostEqual(left_rear, -0.08 / 0.04075)
        self.assertAlmostEqual(right_rear, 0.08 / 0.04075)

    def test_inside_wheels_run_slower_on_left_turn(self):
        _left, _right, left_front, right_front, left_rear, right_rear = self._controller()._targets_from_command(
            self._command(0.12, 0.35),
            fresh=True,
        )
        self.assertLess(abs(left_front), abs(right_front))
        self.assertLess(abs(left_rear), abs(right_rear))

    def test_steering_trim_offsets_zero_steering_when_moving(self):
        controller = self._controller()
        controller.steering_trim = 0.075
        left, right, _left_front, _right_front, _left_rear, _right_rear = controller._targets_from_command(
            self._command(0.12, 0.0),
            fresh=True,
        )
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)

    def test_wheel_speed_scales_can_calibrate_wheel_velocity(self):
        controller = self._controller()
        controller.left_wheel_speed_scale = 0.95
        controller.right_wheel_speed_scale = 1.05
        _left, _right, left_front, right_front, left_rear, right_rear = controller._targets_from_command(
            self._command(0.12, 0.0),
            fresh=True,
        )
        self.assertAlmostEqual(left_front, 0.95 * 0.12 / 0.04075)
        self.assertAlmostEqual(right_front, -1.05 * 0.12 / 0.04075)
        self.assertAlmostEqual(left_rear, 0.95 * 0.12 / 0.04075)
        self.assertAlmostEqual(right_rear, -1.05 * 0.12 / 0.04075)


class ReverseRecoveryPriorityTest(unittest.TestCase):
    def _recovery(self, now=100.0, turn_attempts=8, clear_requested=True, clear_age=12.0):
        recovery = AckermannReverseRecovery.__new__(AckermannReverseRecovery)
        recovery._active_goal_id = "goal"
        recovery._active_goal_since = now - 45.0
        recovery._reverse_count = 0
        recovery.max_reverses_per_goal = 1
        recovery._cooldown_until = 0.0
        recovery._pose = (0.0, 0.0, 0.0)
        recovery._progress_pose = (0.0, 0.0, 0.0)
        recovery._last_progress_wall = now - 32.0
        recovery.min_goal_age = 35.0
        recovery.stuck_timeout = 26.0
        recovery.command_window = 16.0
        recovery.min_forward_attempts = 20
        recovery.min_turn_attempts = 8
        recovery.min_planner_reverse_requests = 10
        recovery.turn_attempt_grace = 22.0
        recovery._clear_requested = clear_requested
        recovery._clear_wall = now - clear_age
        recovery.post_clear_wait = 10.0
        recovery.min_progress_distance = 0.16
        recovery.min_progress_yaw = 0.45
        recovery.yaw_only_progress_enabled = False
        recovery._front_min = float("inf")
        recovery._rear_min = float("inf")
        recovery._last_scan_wall = now
        recovery._scan_angle_span = 6.28
        recovery.rear_clear_distance = 0.75
        recovery.scan_timeout = 0.5
        recovery.require_full_scan_for_reverse = True
        recovery.min_full_scan_angle = 6.0
        recovery.front_obstacle_distance = 0.65
        recovery._front_obstacle_since = None
        recovery.front_obstacle_wait = 12.0
        recovery._last_defer_log = 0.0
        recovery._commands = []
        for index in range(20):
            steering = 0.12 if index < turn_attempts else 0.0
            recovery._commands.append((now - 1.0, 0.05, steering))
        recovery._raw_commands = deque(maxlen=300)
        recovery._defer = lambda *_args: None
        recovery._request_costmap_clear = lambda clear_now: setattr(recovery, "_clear_requested", True)
        return recovery

    def test_reverse_waits_for_turn_attempts_before_replan_window_expires(self):
        now = 100.0
        recovery = self._recovery(now=now, turn_attempts=0)
        recovery._last_progress_wall = now - 30.0
        self.assertFalse(recovery._should_reverse(now))

    def test_reverse_waits_without_forward_or_planner_reverse_evidence(self):
        now = 100.0
        recovery = self._recovery(now=now)
        recovery._commands = []
        self.assertFalse(recovery._should_reverse(now))

    def test_planner_reverse_requests_can_satisfy_exhausted_forward_gate(self):
        now = 100.0
        recovery = self._recovery(now=now)
        recovery._commands = []
        for _index in range(10):
            recovery._raw_commands.append((now - 1.0, -0.01, 0.20))
        self.assertTrue(recovery._should_reverse(now))

    def test_reverse_requests_clear_before_short_reverse(self):
        now = 100.0
        recovery = self._recovery(now=now, clear_requested=False)
        self.assertFalse(recovery._should_reverse(now))
        self.assertTrue(recovery._clear_requested)

    def test_reverse_allowed_after_turning_clear_and_wait(self):
        now = 100.0
        recovery = self._recovery(now=now, clear_requested=True, clear_age=12.0)
        self.assertTrue(recovery._should_reverse(now))

    def test_reverse_recovery_requires_full_scan_coverage(self):
        now = 100.0
        recovery = self._recovery(now=now)
        recovery._scan_angle_span = 4.70
        self.assertFalse(recovery._should_reverse(now))

    def test_reverse_recovery_waits_when_rear_obstacle_is_close(self):
        now = 100.0
        recovery = self._recovery(now=now)
        recovery._rear_min = 0.40
        self.assertFalse(recovery._should_reverse(now))

    def test_yaw_only_motion_does_not_count_as_progress_by_default(self):
        now = 100.0
        recovery = self._recovery(now=now)
        recovery._last_progress_wall = now - 40.0
        recovery._progress_pose = (0.0, 0.0, 0.0)
        recovery._pose = (0.0, 0.0, 1.0)
        recovery._update_progress(now)
        self.assertEqual(recovery._last_progress_wall, now - 40.0)


class TwistToAckermannReverseGateTest(unittest.TestCase):
    def _converter(self, allow_reverse=False, suppressed_reverse_behavior="stop"):
        converter = TwistToAckermann.__new__(TwistToAckermann)
        converter.allow_reverse = allow_reverse
        converter.max_reverse_speed = 0.06
        converter.max_speed = 0.22
        converter.angular_input_mode = "steering_angle"
        converter.suppressed_reverse_behavior = suppressed_reverse_behavior
        return converter

    def test_planner_reverse_is_suppressed_by_default(self):
        speed, suppressed = self._converter(allow_reverse=False)._clamp_speed(-0.05)
        self.assertEqual(speed, 0.0)
        self.assertTrue(suppressed)

    def test_planner_reverse_can_be_enabled_explicitly(self):
        speed, suppressed = self._converter(allow_reverse=True)._clamp_speed(-0.08)
        self.assertAlmostEqual(speed, -0.06)
        self.assertFalse(suppressed)

    def test_suppressed_reverse_stops_without_flipping_teb_steering_by_default(self):
        steering = self._converter()._maybe_flip_suppressed_reverse_steering(0.30, True)
        self.assertAlmostEqual(steering, 0.30)

    def test_forward_crawl_suppressed_reverse_flips_teb_steering(self):
        steering = self._converter(
            suppressed_reverse_behavior="forward_crawl"
        )._maybe_flip_suppressed_reverse_steering(0.30, True)
        self.assertAlmostEqual(steering, -0.30)

    def test_yaw_rate_mode_keeps_yaw_direction_when_reverse_is_suppressed(self):
        converter = self._converter(suppressed_reverse_behavior="forward_crawl")
        converter.angular_input_mode = "yaw_rate"
        steering = converter._maybe_flip_suppressed_reverse_steering(0.30, True)
        self.assertAlmostEqual(steering, 0.30)


class AckermannSafetySupervisorReverseTest(unittest.TestCase):
    def setUp(self):
        rospy_module = AckermannCmdSafetySupervisor.__init__.__globals__["rospy"]
        self._original_logwarn_throttle = rospy_module.logwarn_throttle
        self._original_loginfo_throttle = rospy_module.loginfo_throttle
        rospy_module.logwarn_throttle = lambda *_args, **_kwargs: None
        rospy_module.loginfo_throttle = lambda *_args, **_kwargs: None

    def tearDown(self):
        rospy_module = AckermannCmdSafetySupervisor.__init__.__globals__["rospy"]
        rospy_module.logwarn_throttle = self._original_logwarn_throttle
        rospy_module.loginfo_throttle = self._original_loginfo_throttle

    def _supervisor(self, now=None):
        if now is None:
            now = time.monotonic()
        supervisor = AckermannCmdSafetySupervisor.__new__(AckermannCmdSafetySupervisor)
        supervisor.scan_topic = "/scan"
        supervisor.hard_stop_distance = 0.32
        supervisor.slowdown_distance = 0.95
        supervisor.reaction_time = 0.45
        supervisor.max_deceleration = 0.40
        supervisor.ttc_stop_time = 0.80
        supervisor.scan_timeout = 0.5
        supervisor.require_full_scan_for_reverse = True
        supervisor.min_full_scan_angle = 6.0
        supervisor.frame_id = "base_footprint"
        supervisor.enable_dynamic_escape_boost = True
        supervisor.dynamic_obstacle_timeout = 0.8
        supervisor.escape_prediction_horizon = 1.2
        supervisor.escape_threat_radius = 0.45
        supervisor.escape_speed = 0.30
        supervisor.escape_min_forward_clearance = 1.0
        supervisor.escape_min_obstacle_speed = 0.10
        supervisor._front = DirectionalScanState()
        supervisor._rear = DirectionalScanState()
        supervisor._front.minimum = float("inf")
        supervisor._front.last_scan_wall = now
        supervisor._rear.minimum = float("inf")
        supervisor._rear.last_scan_wall = now
        supervisor._scan_angle_span = 6.28
        supervisor._dynamic_obstacle_message = None
        supervisor._dynamic_obstacle_wall = 0.0
        supervisor._tf = None
        return supervisor

    @staticmethod
    def _dynamic_obstacle_message(x, y, radius, vx, vy, frame_id="base_footprint"):
        message = ObstacleArrayMsg()
        message.header.frame_id = frame_id
        obstacle = ObstacleMsg()
        obstacle.id = 7
        point = Point32()
        point.x = x
        point.y = y
        obstacle.polygon.points = [point]
        obstacle.radius = radius
        obstacle.velocities.twist.linear.x = vx
        obstacle.velocities.twist.linear.y = vy
        message.obstacles.append(obstacle)
        return message

    def test_reverse_command_is_stopped_without_full_scan_coverage(self):
        supervisor = self._supervisor()
        supervisor._scan_angle_span = 4.70
        command = AckermannDriveStamped()
        command.drive.speed = -0.03

        self.assertTrue(supervisor._should_emergency_stop(command))

    def test_reverse_command_stops_for_rear_obstacle(self):
        supervisor = self._supervisor()
        supervisor._rear.minimum = 0.25
        command = AckermannDriveStamped()
        command.drive.speed = -0.03

        self.assertTrue(supervisor._should_emergency_stop(command))

    def test_reverse_slowdown_uses_rear_clearance(self):
        supervisor = self._supervisor()
        supervisor._rear.minimum = 0.50
        command = AckermannDriveStamped()
        command.drive.speed = -0.06

        slowed = supervisor._apply_slowdown(command)

        self.assertLess(abs(slowed.drive.speed), abs(command.drive.speed))
        self.assertLess(slowed.drive.speed, 0.0)

    def test_dynamic_escape_boosts_forward_command_on_crossing_obstacle_path(self):
        supervisor = self._supervisor()
        supervisor._front.minimum = 2.0
        supervisor._dynamic_obstacle_callback(
            self._dynamic_obstacle_message(-0.40, 0.0, 0.20, 0.45, 0.0)
        )
        command = AckermannDriveStamped()
        command.drive.speed = 0.12

        boosted = supervisor._apply_dynamic_escape_boost(command)

        self.assertAlmostEqual(boosted.drive.speed, 0.30)

    def test_dynamic_escape_does_not_boost_far_obstacle_path(self):
        supervisor = self._supervisor()
        supervisor._front.minimum = 2.0
        supervisor._dynamic_obstacle_callback(
            self._dynamic_obstacle_message(-2.00, 0.0, 0.20, 0.30, 0.0)
        )
        command = AckermannDriveStamped()
        command.drive.speed = 0.12

        boosted = supervisor._apply_dynamic_escape_boost(command)

        self.assertAlmostEqual(boosted.drive.speed, 0.12)

    def test_dynamic_escape_waits_when_forward_clearance_is_low(self):
        supervisor = self._supervisor()
        supervisor._front.minimum = 0.80
        supervisor._dynamic_obstacle_callback(
            self._dynamic_obstacle_message(-0.40, 0.0, 0.20, 0.45, 0.0)
        )
        command = AckermannDriveStamped()
        command.drive.speed = 0.12

        boosted = supervisor._apply_dynamic_escape_boost(command)

        self.assertAlmostEqual(boosted.drive.speed, 0.12)

    def test_dynamic_escape_does_not_boost_reverse_command(self):
        supervisor = self._supervisor()
        supervisor._front.minimum = 2.0
        supervisor._dynamic_obstacle_callback(
            self._dynamic_obstacle_message(-0.40, 0.0, 0.20, 0.45, 0.0)
        )
        command = AckermannDriveStamped()
        command.drive.speed = -0.03

        boosted = supervisor._apply_dynamic_escape_boost(command)

        self.assertAlmostEqual(boosted.drive.speed, -0.03)


if __name__ == "__main__":
    unittest.main()
