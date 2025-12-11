#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

FREE_THRESHOLD = 254
OCC_THRESHOLD = 205

def make_occupancy_grid():
    # Load grayscale map (0–255)
    script_dir = os.path.dirname(os.path.realpath(__file__))
    img_path = os.path.join(script_dir, "maps", "project_map.pgm")
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("Could not open maps/project_map.pgm")

    img = cv2.flip(img, 0)   # 0 = flip around x-axis

    #Create occupancy grid
    occ_grid = np.full(img.shape, -1, dtype=np.int8)
    occ_grid[img >= FREE_THRESHOLD] = 0
    occ_grid[img <= OCC_THRESHOLD] = 100

    return occ_grid

def publish_map():
    rospy.init_node("map_publisher")

    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)

    grid = make_occupancy_grid()

    msg = OccupancyGrid()
    msg.header.frame_id = "odom"

    h, w = grid.shape
    msg.info.width = w
    msg.info.height = h
    msg.info.resolution = 0.05  

    # Origin: bottom-left of map in world coordinates
    msg.info.origin = Pose()
    msg.info.origin.position.x = -10.0
    msg.info.origin.position.y = -10.0
    msg.info.origin.orientation.w = 1.0

    # Flatten row-major for ROS
    msg.data = grid.flatten().tolist()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    publish_map()
