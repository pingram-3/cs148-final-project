#!/usr/bin/env python3
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

class TFPoseHelper:
    def __init__(self, fixed_frame="odom", base_frame="base_footprint"):
        self.fixed_frame = fixed_frame
        self.base_frame = base_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_robot_pose(self):
        """
        Returns (x, y, yaw) of base_frame in fixed_frame, or None on failure.
        """
        try:
            # Time(0) => "latest available"
            trans = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed for %s -> %s", self.fixed_frame, self.base_frame)
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        q = trans.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        return (x, y, yaw)
