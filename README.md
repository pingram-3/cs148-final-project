Final project for CS148

Paul Ingram and Draylend Chow

## Installation

Clone this package into your `catkin_ws/src`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/pingram-3/cs148-final-project.git
mv ~/catkin_ws/src/cs148-final-project ~/catkin_ws/src/cs148_final_project
cd ..
catkin_make
source devel/setup.bash
```

## Example Usage
### Set Up
```bash
roscore # Terminal 1; start ROS master
roslaunch turtlebot3_gazebo turtlebot3_world.launch # Terminal 2; launch Gazebo
rosrun cs148_final_project occupancy_grid.py    # Terminal 3
rosrun cs148_final_project dijkstra_planner.py  # Terminal 4
rosrun cs148_final_project path_follower.py     # Terminal 5
```

## Using the Program
```bash
# Launch rviz
rviz
```