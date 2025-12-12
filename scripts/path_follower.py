#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

from tf_pose import TFPoseHelper  # same helper you already use


class PathFollower:
    def __init__(self):
        # Params
        self.fixed_frame = rospy.get_param("~fixed_frame", "odom")
        self.base_frame  = rospy.get_param("~base_frame", "base_footprint")

        self.max_lin_speed   = rospy.get_param("~max_lin_speed", 0.2)   # m/s
        self.max_ang_speed   = rospy.get_param("~max_ang_speed", 0.8)   # rad/s
        self.k_lin           = rospy.get_param("~k_lin", 0.8)
        self.k_ang           = rospy.get_param("~k_ang", 2.0)
        self.waypoint_tol    = rospy.get_param("~waypoint_tolerance", 0.1)   # m
        self.heading_tol     = rospy.get_param("~heading_tolerance", 0.3)    # rad

        self.pose_helper = TFPoseHelper(
            fixed_frame=self.fixed_frame,
            base_frame=self.base_frame,
        )

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Create priority parameters
        # Always follow A* path, if there isn't one, follow Dijkstra, if there isn't one, then follow greedy
        self.astar_path = None
        self.dij_path   = None
        self.best_first_path = None
        self.last_astar_time = 0.0

        # Subscribe to all topics of path planning
        rospy.Subscriber("/a_star_path",   Path, self.a_star_callback,   queue_size=1)
        rospy.Subscriber("/dijkstra_path", Path, self.dijkstra_callback, queue_size=1)
        rospy.Subscriber("/best_first_path",  Path, self.best_first_callback,  queue_size=1)

        # Internal state
        self.waypoints = []
        self.current_idx = 0
        self.active = False
        self.current_priority = None

        rospy.loginfo("PathFollower with priority: A* > Dijkstra > Greedy")

    # Fill in values for paths (check if there is a generated path from the topic node)
    # PRIORITY: 1 = A*, 2 = Dijkstra, 3 = Best-first
    def a_star_callback(self, msg):
        if not msg.poses:
            return

        # Always override - highest priority
        self.current_priority = 1
        self.astar_path = msg
        self._load_new_path(msg, "A*")

    def dijkstra_callback(self, msg):
        if not msg.poses:
            return

        # Only override if no A* path is active
        if self.current_priority is None or self.current_priority > 1:
            self.current_priority = 2
            self.dij_path = msg
            self._load_new_path(msg, "Dijkstra")

    def best_first_callback(self, msg):
        if not msg.poses:
            return

        # Only override if no A* or Dijkstra path is active
        if self.current_priority is None or self.current_priority > 2:
            self.current_priority = 3
            self.best_first_path = msg
            self._load_new_path(msg, "Best-First")

    def _load_new_path(self, msg, name="Unknown"):
        self.waypoints = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.current_idx = 0
        self.active = True
        rospy.loginfo("Using %s path with %d waypoints", name, len(self.waypoints))

    def stop(self):
        self.active = False
        twist = Twist()
        self.cmd_pub.publish(twist)

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update(self):
        """Control loop step."""
        if not self.active or not self.waypoints:
            # Nothing to do, ensure robot is stopped
            self.stop()
            return

        pose = self.pose_helper.get_robot_pose()
        if pose is None:
            rospy.logwarn_throttle(1.0, "No TF pose in PathFollower")
            self.stop()
            return

        x, y, yaw = pose

        # If we ran out of waypoints, stop
        if self.current_idx >= len(self.waypoints):
            rospy.loginfo("Reached end of path, stopping")
            self.stop()
            return

        goal_x, goal_y = self.waypoints[self.current_idx]

        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)

        # If close enough to this waypoint, move to next
        if dist < self.waypoint_tol:
            rospy.loginfo("Waypoint %d reached (dist=%.3f)", self.current_idx, dist)
            self.current_idx += 1
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self._wrap_angle(target_yaw - yaw)

        twist = Twist()

        # Rotate to face waypoint first
        if abs(yaw_err) > self.heading_tol:
            twist.linear.x = 0.0
            twist.angular.z = max(-self.max_ang_speed,
                                  min(self.max_ang_speed, self.k_ang * yaw_err))
        else:
            # Move toward waypoint while keeping heading correction
            twist.linear.x = min(self.max_lin_speed, self.k_lin * dist)
            twist.angular.z = max(-self.max_ang_speed,
                                  min(self.max_ang_speed, self.k_ang * yaw_err))

        self.cmd_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("path_follower")

    follower = PathFollower()
    rate = rospy.Rate(20)  # 20 Hz control loop

    rospy.loginfo("PathFollower node started")

    while not rospy.is_shutdown():
        follower.update()
        rate.sleep()

    follower.stop()
