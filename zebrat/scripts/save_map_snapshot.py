#!/usr/bin/env python3

import hashlib
import os
import time

import rospy
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion


class MapSnapshotSaver:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.output_prefix = rospy.get_param("~output_prefix")
        self.stable_duration = float(rospy.get_param("~idle_duration", 8.0))
        self.max_wait_duration = float(rospy.get_param("~max_wait_duration", 180.0))
        self.min_known_cells = int(rospy.get_param("~min_known_cells", 100))
        self.min_width = int(rospy.get_param("~min_width", 20))
        self.min_height = int(rospy.get_param("~min_height", 20))
        self.latest_map = None
        self.first_ready_wall = None
        self.last_changed_wall = None
        self.last_signature = None
        self.saved = False

        if not self.output_prefix:
            raise rospy.ROSInitException("~output_prefix is required")

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self._timer_callback)

    def _map_callback(self, message):
        self.latest_map = message
        if not self._map_is_ready(message):
            return

        now = time.monotonic()
        signature = self._map_signature(message)
        if self.first_ready_wall is None:
            self.first_ready_wall = now
        if signature != self.last_signature:
            self.last_signature = signature
            self.last_changed_wall = now

    def _map_is_ready(self, message):
        if message is None:
            return False
        if message.info.width < self.min_width or message.info.height < self.min_height:
            return False
        known_cells = sum(1 for value in message.data if value >= 0)
        return known_cells >= self.min_known_cells

    def _map_signature(self, message):
        digest = hashlib.blake2b(digest_size=12)
        digest.update(str(message.info.width).encode("ascii"))
        digest.update(str(message.info.height).encode("ascii"))
        digest.update(f"{message.info.resolution:.6f}".encode("ascii"))
        digest.update(f"{message.info.origin.position.x:.6f}".encode("ascii"))
        digest.update(f"{message.info.origin.position.y:.6f}".encode("ascii"))
        digest.update(bytes((value + 1) & 0xFF for value in message.data))
        return digest.hexdigest()

    def _timer_callback(self, _event):
        if self.saved or self.latest_map is None or self.first_ready_wall is None:
            return
        if not self._map_is_ready(self.latest_map):
            return

        now = time.monotonic()
        stable_for = now - (self.last_changed_wall or self.first_ready_wall)
        waited_for = now - self.first_ready_wall
        if stable_for < self.stable_duration and waited_for < self.max_wait_duration:
            return

        if waited_for >= self.max_wait_duration:
            rospy.logwarn(
                "Saving map snapshot after max wait %.1fs even though changes are still arriving",
                self.max_wait_duration,
            )
        self._save(self.latest_map)
        self.saved = True
        rospy.signal_shutdown("map snapshot saved")

    def _save(self, message):
        directory = os.path.dirname(self.output_prefix)
        if directory:
            os.makedirs(directory, exist_ok=True)

        pgm_path = self.output_prefix + ".pgm"
        yaml_path = self.output_prefix + ".yaml"
        width = message.info.width
        height = message.info.height
        pixels = bytearray()

        for row in range(height - 1, -1, -1):
            row_offset = row * width
            for col in range(width):
                value = message.data[row_offset + col]
                if value < 0:
                    pixels.append(205)
                elif value >= 65:
                    pixels.append(0)
                else:
                    pixels.append(254)

        with open(pgm_path, "wb") as pgm_file:
            pgm_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            pgm_file.write(pixels)

        orientation = message.info.origin.orientation
        yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]

        yaml_contents = (
            f"image: {os.path.basename(pgm_path)}\n"
            f"resolution: {message.info.resolution:.6f}\n"
            f"origin: [{message.info.origin.position.x:.6f}, "
            f"{message.info.origin.position.y:.6f}, {yaw:.6f}]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        with open(yaml_path, "w", encoding="utf-8") as yaml_file:
            yaml_file.write(yaml_contents)

        rospy.loginfo("Saved map snapshot to %s(.pgm/.yaml)", self.output_prefix)


def main():
    rospy.init_node("save_map_snapshot")
    MapSnapshotSaver()
    rospy.spin()


if __name__ == "__main__":
    main()
