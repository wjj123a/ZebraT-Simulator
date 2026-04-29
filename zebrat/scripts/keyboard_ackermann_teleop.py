#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


BANNER = """
Reading from the keyboard and publishing {topic} (ackermann_msgs/AckermannDriveStamped).
---------------------------
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

KEY_BINDINGS = {
    "w": (1.0, 0.0),
    "a": (1.0, 1.0),
    "d": (1.0, -1.0),
    "s": (-1.0, 0.0),
    "z": (0.0, 0.0),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_command(speed, steering, frame_id):
    command = AckermannDriveStamped()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = frame_id
    command.drive.speed = speed
    command.drive.steering_angle = steering
    return command


def main():
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("ackermann_keyop")
    output_topic = rospy.get_param("~output_topic", "/ackermann_cmd_teleop")
    frame_id = rospy.get_param("~frame_id", "base_footprint")
    speed = float(rospy.get_param("~speed", 0.22))
    steering = float(rospy.get_param("~steering_angle", 0.45))
    publisher = rospy.Publisher(output_topic, AckermannDriveStamped, queue_size=1)
    print(BANNER.format(topic=output_topic))

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key in KEY_BINDINGS:
                speed_scale, steering_scale = KEY_BINDINGS[key]
            else:
                speed_scale, steering_scale = 0.0, 0.0
                if key == "\x03":
                    break

            publisher.publish(make_command(speed_scale * speed, steering_scale * steering, frame_id))
    finally:
        publisher.publish(make_command(0.0, 0.0, frame_id))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
