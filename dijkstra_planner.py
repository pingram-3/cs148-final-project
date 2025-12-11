#!/usr/bin/env python3
from dijkstra import dijkstra
from tf_pose import TFPoseHelper
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import rospy
import numpy as np

FREE = 0
OCCUPIED = 100

class DijkstraPlanner:
    def __init__(self, fixed_frame="map", base_frame="base_link"):
        self.map_sub  = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher("/dijkstra_path", Path, queue_size=1, latch=True)

        self.grid = None
        self.info = None

        # TF pose helper
        self.pose_helper = TFPoseHelper(fixed_frame=fixed_frame,
                                        base_frame=base_frame)

    def map_callback(self, msg):
        self.info = msg.info
        data = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        # 0 = free, 100 = occupied, -1 = unknown
        grid = np.zeros_like(data, dtype=np.int8)
        grid[data == OCCUPIED] = 1   # only 100 is blocked
        grid[data != OCCUPIED] = 0   # treat unknown (-1) as free for now

        self.grid = grid

    def world_to_grid(self, x, y):
        """
        Convert world (map frame) coordinates to grid indices (gx, gy).
        """
        res = self.info.resolution
        origin_x = self.info.origin.position.x
        origin_y = self.info.origin.position.y

        gx = int((x - origin_x) / res)
        gy = int((y - origin_y) / res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """
        Convert grid indices (gx, gy) to world (map frame) coordinates.
        """
        res = self.info.resolution
        origin_x = self.info.origin.position.x
        origin_y = self.info.origin.position.y

        x = gx * res + origin_x + res / 2.0
        y = gy * res + origin_y + res / 2.0
        return x, y

    def plan_and_publish(self, start_world, goal_world):
        if self.grid is None or self.info is None:
            rospy.logwarn("No map received yet")
            return

        # Convert world → grid
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        h, w = self.grid.shape

        # Debug logs (you’re already seeing these)
        rospy.loginfo(
            f"Start world: {start_world} -> grid: ({sx}, {sy}), value={self.grid[sy, sx]}"
        )
        rospy.loginfo(
            f"Goal  world: {goal_world} -> grid: ({gx}, {gy}), value={self.grid[gy, gx]}"
        )

        # Bounds check
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            rospy.logerr("Start or goal out of bounds")
            return

        # Obstacle check (0 = free, anything else = not free)
        if self.grid[sy, sx] != 0 or self.grid[gy, gx] != 0:
            rospy.logerr("Start or goal in obstacle")
            return

        # --- Run Dijkstra on the grid ---
        path_cells = dijkstra(self.grid, (sx, sy), (gx, gy))
        if not path_cells:
            rospy.logwarn("No path found by Dijkstra")
            return

        rospy.loginfo(f"Dijkstra found path with {len(path_cells)} cells")

        # --- Build Path message ---
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"  # since we’re in the odom frame

        for (cx, cy) in path_cells:
            wx, wy = self.grid_to_world(cx, cy)
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published Dijkstra path with %d points", len(path_msg.poses))

    
    def plan_from_robot_to_goal(self, goal_world):
        if self.grid is None or self.info is None:
            rospy.logwarn("No map yet")
            return

        pose = self.pose_helper.get_robot_pose()
        if pose is None:
            rospy.logwarn("No TF pose yet")
            return

        x, y, yaw = pose
        start_world = (x, y)

        self.plan_and_publish(start_world, goal_world)

if __name__ == "__main__":
    rospy.init_node("dijkstra_planner_tf")

    # Change fixed_frame if you don’t have map
    planner = DijkstraPlanner(fixed_frame="odom", base_frame="base_footprint")

    rospy.loginfo("Waiting for map and TF pose...")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if planner.grid is not None:
            pose = planner.pose_helper.get_robot_pose()
            if pose is not None:
                rospy.loginfo("Map and TF pose ready")
                break
        rate.sleep()

    if rospy.is_shutdown():
        raise SystemExit

     # Current robot pose in odom
    x, y, yaw = planner.pose_helper.get_robot_pose()

    # For now: simple goal 1 m in front in odom frame
    goal_world = (x + 1.0, y)

    planner.plan_and_publish((x, y), goal_world)

    rospy.spin()