#!/usr/bin/env python3

import html
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


class DynamicObstacle:
    def __init__(self, spec, speed_scale):
        self.name = str(spec["name"])
        self.kind = str(spec.get("type", "cart"))
        self.loop = str(spec.get("loop", "ping_pong"))
        self.speed = max(0.01, float(spec.get("speed", 0.4)) * speed_scale)
        self.spec = spec
        self.waypoints = [self._parse_waypoint(item, spec) for item in spec.get("waypoints", [])]
        if len(self.waypoints) < 2:
            raise ValueError("Obstacle %s must define at least two waypoints" % self.name)

        start_index = int(spec.get("start_index", 0)) % len(self.waypoints)
        self.x, self.y, self.z = self.waypoints[start_index]
        self.target_index = (start_index + 1) % len(self.waypoints)
        self.direction = 1
        self.yaw = self._yaw_to_target()

    @staticmethod
    def _parse_waypoint(item, spec):
        if isinstance(item, dict):
            return (
                float(item["x"]),
                float(item["y"]),
                float(item.get("z", spec.get("z", 0.0))),
            )
        return (
            float(item[0]),
            float(item[1]),
            float(item[2]) if len(item) > 2 else float(spec.get("z", 0.0)),
        )

    def _yaw_to_target(self):
        tx, ty, _tz = self.waypoints[self.target_index]
        return math.atan2(ty - self.y, tx - self.x)

    def _advance_target(self):
        if self.loop == "cycle":
            self.target_index = (self.target_index + 1) % len(self.waypoints)
            return

        if self.target_index >= len(self.waypoints) - 1:
            self.direction = -1
        elif self.target_index <= 0:
            self.direction = 1
        self.target_index += self.direction

    def update(self, dt):
        remaining_step = self.speed * max(0.0, dt)
        while remaining_step > 0.0 and not rospy.is_shutdown():
            tx, ty, tz = self.waypoints[self.target_index]
            dx = tx - self.x
            dy = ty - self.y
            dz = tz - self.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < 1e-6:
                self._advance_target()
                continue

            self.yaw = math.atan2(dy, dx)
            if remaining_step >= distance:
                self.x, self.y, self.z = tx, ty, tz
                remaining_step -= distance
                self._advance_target()
            else:
                ratio = remaining_step / distance
                self.x += dx * ratio
                self.y += dy * ratio
                self.z += dz * ratio
                remaining_step = 0.0

    def pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose


class DynamicObstacleController:
    def __init__(self):
        self.profile_name = rospy.get_param("~profile", "area_mixed")
        self.speed_scale = float(rospy.get_param("~speed_scale", 1.0))
        profiles = rospy.get_param("~profiles", {})
        if self.profile_name not in profiles:
            raise rospy.ROSInitException("Dynamic obstacle profile '%s' was not found" % self.profile_name)

        self.profile = profiles[self.profile_name]
        self.update_rate = float(self.profile.get("update_rate", 12.0))
        self.respawn_existing = bool(self.profile.get("respawn_existing", True))
        self.cleanup_on_shutdown = bool(self.profile.get("cleanup_on_shutdown", True))
        self.obstacles = [
            DynamicObstacle(spec, self.speed_scale)
            for spec in self.profile.get("obstacles", [])
        ]
        if not self.obstacles:
            raise rospy.ROSInitException("Dynamic obstacle profile '%s' has no obstacles" % self.profile_name)

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/get_world_properties")
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        rospy.on_shutdown(self.cleanup)

    @staticmethod
    def _rgba(spec, default):
        values = spec.get("color", default)
        return "%s %s %s %s" % (
            float(values[0]),
            float(values[1]),
            float(values[2]),
            float(values[3]) if len(values) > 3 else 1.0,
        )

    def _pedestrian_sdf(self, obstacle):
        radius = float(obstacle.spec.get("radius", 0.24))
        height = float(obstacle.spec.get("height", 1.6))
        color = self._rgba(obstacle.spec, [0.2, 0.32, 0.86, 1.0])
        name = html.escape(obstacle.name)
        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%s">
    <static>false</static>
    <link name="base_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial>
        <mass>70.0</mass>
        <inertia>
          <ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 %.3f 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>%.3f</radius>
            <length>%.3f</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="sensor_body">
        <pose>0 0 %.3f 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>%.3f</radius>
            <length>%.3f</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>%s</ambient>
          <diffuse>%s</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""" % (name, height / 2.0, radius, height, height / 2.0, radius, height, color, color)

    def _vehicle_sdf(self, obstacle):
        size = obstacle.spec.get("size", [0.9, 0.5, 0.55])
        length = float(size[0])
        width = float(size[1])
        height = float(size[2])
        color = self._rgba(obstacle.spec, [0.95, 0.55, 0.16, 1.0])
        name = html.escape(obstacle.name)
        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%s">
    <static>false</static>
    <link name="base_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial>
        <mass>40.0</mass>
        <inertia>
          <ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 %.3f 0 0 0</pose>
        <geometry>
          <box>
            <size>%.3f %.3f %.3f</size>
          </box>
        </geometry>
      </collision>
      <visual name="body">
        <pose>0 0 %.3f 0 0 0</pose>
        <geometry>
          <box>
            <size>%.3f %.3f %.3f</size>
          </box>
        </geometry>
        <material>
          <ambient>%s</ambient>
          <diffuse>%s</diffuse>
        </material>
      </visual>
      <visual name="front_marker">
        <pose>%.3f 0 %.3f 0 0 0</pose>
        <geometry>
          <box>
            <size>0.06 %.3f %.3f</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""" % (
            name,
            height / 2.0,
            length,
            width,
            height,
            height / 2.0,
            length,
            width,
            height,
            color,
            color,
            length / 2.0 + 0.031,
            height / 2.0,
            width,
            height,
        )

    def _sdf_for(self, obstacle):
        if obstacle.kind == "pedestrian":
            return self._pedestrian_sdf(obstacle)
        return self._vehicle_sdf(obstacle)

    def spawn_all(self):
        existing_models = set(self.get_world_properties().model_names)
        for obstacle in self.obstacles:
            if self.respawn_existing and obstacle.name in existing_models:
                try:
                    self.delete_model(obstacle.name)
                    time.sleep(0.05)
                except rospy.ServiceException:
                    pass

            response = self.spawn_model(obstacle.name, self._sdf_for(obstacle), "", obstacle.pose(), "world")
            if not response.success:
                raise rospy.ROSException("Failed to spawn %s: %s" % (obstacle.name, response.status_message))
            existing_models.add(obstacle.name)
            rospy.loginfo("Spawned dynamic obstacle %s", obstacle.name)

    def publish_state(self, obstacle):
        state = ModelState()
        state.model_name = obstacle.name
        state.pose = obstacle.pose()
        state.reference_frame = "world"
        try:
            response = self.set_model_state(state)
        except rospy.ServiceException as exc:
            if not rospy.is_shutdown():
                rospy.logwarn_throttle(5.0, "Failed to move %s: %s", obstacle.name, exc)
            return
        if not response.success:
            rospy.logwarn_throttle(5.0, "Failed to move %s: %s", obstacle.name, response.status_message)

    def run(self):
        self.spawn_all()
        period = 1.0 / max(1.0, self.update_rate)
        last_time = time.monotonic()
        while not rospy.is_shutdown():
            loop_start = time.monotonic()
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            for obstacle in self.obstacles:
                obstacle.update(dt)
                self.publish_state(obstacle)
            time.sleep(max(0.0, period - (time.monotonic() - loop_start)))

    def cleanup(self):
        if not self.cleanup_on_shutdown:
            return
        for obstacle in self.obstacles:
            try:
                self.delete_model(obstacle.name)
                rospy.loginfo("Deleted dynamic obstacle %s", obstacle.name)
            except Exception:
                pass


def main():
    rospy.init_node("dynamic_obstacles")
    controller = DynamicObstacleController()
    controller.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        if not rospy.is_shutdown():
            rospy.logerr("Dynamic obstacle controller failed: %s", exc)
            sys.exit(1)
