#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

"""
Legacy compatibility bridge.
Republishes Twist commands to the historical ZebraT topic.
"""

class TwistRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/cmd_vel")
        output_topic = rospy.get_param("~output_topic", "/ackermann_cmd_mux/output")
        self.publisher = rospy.Publisher(output_topic, Twist, queue_size=1)
        rospy.Subscriber(input_topic, Twist, self.callback, queue_size=1)

    def callback(self, msg):
        self.publisher.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('nav_sim', anonymous=True)
        TwistRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
