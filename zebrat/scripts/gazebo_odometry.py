#!/usr/bin/env python3

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

class OdometryNode:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "r1")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_footprint")
        self.base_link_name = rospy.get_param("~base_link_name", "base_link")
        self.base_footprint_name = rospy.get_param("~base_footprint_name", "base_footprint")
        self.base_link_z_offset = float(rospy.get_param("~base_link_z_offset", 0.04075))
        self.project_to_ground = bool(rospy.get_param("~project_to_ground", True))
        self.zero_lateral_twist = bool(rospy.get_param("~zero_lateral_twist", True))

        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set publishers
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def _qualified_name(self, link_name):
        return "%s::%s" % (self.model_name, link_name)

    def _pose_as_base_footprint(self, pose, link_name):
        if link_name == self.base_footprint_name:
            return pose

        corrected = Pose()
        quaternion = pose.orientation
        rotation = quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        offset = rotation[:3, :3].dot(np.array([0.0, 0.0, self.base_link_z_offset]))
        corrected.position.x = pose.position.x - offset[0]
        corrected.position.y = pose.position.y - offset[1]
        corrected.position.z = pose.position.z - offset[2]
        corrected.orientation = pose.orientation
        if self.project_to_ground:
            corrected.position.z = 0.0
            _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            yaw_only = quaternion_from_euler(0.0, 0.0, yaw)
            corrected.orientation.x = yaw_only[0]
            corrected.orientation.y = yaw_only[1]
            corrected.orientation.z = yaw_only[2]
            corrected.orientation.w = yaw_only[3]
        return corrected

    def _twist_as_base_footprint(self, twist, pose, link_name):
        if not self.project_to_ground:
            return twist

        quaternion = pose.orientation
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        corrected = Twist()
        corrected.linear.x = cos_yaw * twist.linear.x + sin_yaw * twist.linear.y
        corrected.linear.y = 0.0 if self.zero_lateral_twist else -sin_yaw * twist.linear.x + cos_yaw * twist.linear.y
        corrected.linear.z = 0.0
        corrected.angular.x = 0.0
        corrected.angular.y = 0.0
        corrected.angular.z = twist.angular.z
        return corrected

    def sub_robot_pose_update(self, msg):
        for link_name in (self.base_link_name, self.base_footprint_name):
            qualified = self._qualified_name(link_name)
            try:
                arrayIndex = msg.name.index(qualified)
            except ValueError:
                continue

            self.last_received_pose = self._pose_as_base_footprint(msg.pose[arrayIndex], link_name)
            self.last_received_twist = self._twist_as_base_footprint(
                msg.twist[arrayIndex],
                msg.pose[arrayIndex],
                link_name,
            )
            self.last_recieved_stamp = rospy.Time.now()
            return

    def timer_callback(self, event):
        if rospy.is_shutdown():
            return

        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = self.odom_frame_id
        cmd.child_frame_id = self.child_frame_id
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3]

        cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]

        try:
            self.pub_odom.publish(cmd)
        except rospy.ROSException:
            return

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        try:
            self.tf_pub.sendTransform(tf)
        except rospy.ROSException:
            return

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
