#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

"""
Legacy compatibility bridge.
Accepts the historical ZebraT command topic and republishes to /cmd_vel.
"""


class TwistRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/ackermann_cmd_mux/output")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.publisher = rospy.Publisher(output_topic, Twist, queue_size=1)
        rospy.Subscriber(input_topic, Twist, self.callback, queue_size=1)

    def callback(self, msg):
        self.publisher.publish(msg)


def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)
    TwistRelay()
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
