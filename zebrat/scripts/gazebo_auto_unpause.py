#!/usr/bin/env python3

import rospy
import time
from gazebo_msgs.srv import GetPhysicsProperties
from std_srvs.srv import Empty


class GazeboAutoUnpause:
    def __init__(self):
        self.check_period = float(rospy.get_param("~check_period", 1.0))
        self.keep_unpaused = bool(rospy.get_param("~keep_unpaused", True))
        self.max_wait = float(rospy.get_param("~max_wait", 60.0))
        self._unpause = None
        self._get_physics = None

    def _connect(self):
        rospy.loginfo("Waiting for Gazebo physics services")
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=self.max_wait)
        rospy.wait_for_service("/gazebo/get_physics_properties", timeout=self.max_wait)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self._get_physics = rospy.ServiceProxy(
            "/gazebo/get_physics_properties", GetPhysicsProperties
        )

    def _unpause_if_needed(self):
        try:
            state = self._get_physics()
            if state.pause:
                self._unpause()
                rospy.loginfo("Gazebo physics was paused; requested unpause")
            return state.pause
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "Failed to query/unpause Gazebo physics: %s", exc)
            return True

    def run(self):
        try:
            self._connect()
        except rospy.ROSException as exc:
            rospy.logerr("Timed out waiting for Gazebo physics services: %s", exc)
            return

        while not rospy.is_shutdown():
            was_paused = self._unpause_if_needed()
            if not self.keep_unpaused and not was_paused:
                return
            # Use wall time instead of rospy.Rate: /use_sim_time stops advancing
            # while Gazebo is paused, exactly when this watchdog must keep retrying.
            time.sleep(max(self.check_period, 0.1))


def main():
    rospy.init_node("gazebo_auto_unpause")
    GazeboAutoUnpause().run()


if __name__ == "__main__":
    main()
