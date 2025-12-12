#!/usr/bin/env python3
from a_star import astar
from tf_pose import TFPoseHelper
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import rospy
import numpy as np

FREE = 0
OCCUPIED = 100

class AStarPlanner:
    def __init__(self, fixed_frame="odom", base_frame="base_footprint"):
        self.map_sub  = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher("/a_star_path", Path, queue_size=1, latch=True)

        # Helper to get robot pose in the same frame as RViz goals
        self.pose_helper = TFPoseHelper(fixed_frame=fixed_frame,
                                        base_frame=base_frame)

        self.grid = None      
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        
         # subscribe to RViz goals
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal",    # topic RViz uses for 2D Nav Goal
            PoseStamped,
            self.goal_callback,
        )
        
        self.goal_sub_rviz = rospy.Subscriber(
            "/rviz/move_base_simple/goal",
            PoseStamped,
            self.goal_callback,
            queue_size=1,
        )

    def map_callback(self, msg):
        self.info = msg.info
        data = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        # 0 = free, 100 = occupied, -1 = unknown
        grid = np.zeros_like(data, dtype=np.int8)
        grid[data == OCCUPIED] = 1   # only 100 is blocked
        grid[data != OCCUPIED] = 0   

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

        # Debug logs 
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

        # --- Run A* on the grid ---
        path_cells = astar(self.grid, (sx, sy), (gx, gy))
        if not path_cells:
            rospy.logwarn("No path found by A*")
            return

        rospy.loginfo(f"A* found path with {len(path_cells)} cells")

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
        rospy.loginfo("Published A* path with %d points", len(path_msg.poses))

    
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

    def goal_callback(self, msg: PoseStamped):
        # Make sure we have a map
        if self.grid is None:
            rospy.logwarn("No map received yet, cannot plan")
            return

        # Get current robot pose
        pose = self.pose_helper.get_robot_pose()
        if pose is None:
            rospy.logwarn("Cannot get robot pose, skipping goal")
            return

        x, y, yaw = pose

        # Goal from RViz click (in map frame)
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        rospy.loginfo(f"Received RViz goal: ({gx:.3f}, {gy:.3f})")

        # Call your existing planner function that expects world coords
        self.plan_and_publish((x, y), (gx, gy))

if __name__ == "__main__":
    rospy.init_node("a_star_planner_tf") 

    # Use frame RViz goals are in
    planner = AStarPlanner(fixed_frame="odom", base_frame="base_footprint")

    rospy.loginfo("A*Planner waiting for goals from RViz (/move_base_simple/goal)")
    rospy.spin()