#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler


class InitialPosePublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.x = float(rospy.get_param("~x", 0.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.yaw = float(rospy.get_param("~yaw", 0.0))
        self.x_stddev = float(rospy.get_param("~x_stddev", 0.2))
        self.y_stddev = float(rospy.get_param("~y_stddev", 0.2))
        self.yaw_stddev = float(rospy.get_param("~yaw_stddev", 0.15))
        self.publish_count = int(rospy.get_param("~publish_count", 5))
        self.publish_interval = float(rospy.get_param("~publish_interval", 0.5))
        self.start_delay = float(rospy.get_param("~start_delay", 2.0))

        self._published = 0
        self._publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        rospy.Timer(rospy.Duration(self.start_delay), self._start_timer, oneshot=True)

    def _start_timer(self, _event):
        rospy.Timer(rospy.Duration(self.publish_interval), self._publish_timer)

    def _publish_timer(self, event):
        if self._published >= self.publish_count:
            rospy.signal_shutdown("initial pose published")
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        quaternion = quaternion_from_euler(0.0, 0.0, self.yaw)
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.pose.covariance[0] = self.x_stddev ** 2
        message.pose.covariance[7] = self.y_stddev ** 2
        message.pose.covariance[35] = self.yaw_stddev ** 2

        self._publisher.publish(message)
        self._published += 1


def main():
    rospy.init_node("publish_initial_pose")
    InitialPosePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
