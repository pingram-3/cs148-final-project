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
### Set Up (automated)
```bash
roslaunch cs148_final_project path_planning_robot.launch
```

### Set Up (manual)
```bash
roscore # Terminal 1; start ROS master
roslaunch turtlebot3_gazebo turtlebot3_world.launch # Terminal 2; launch Gazebo
rosrun cs148_final_project occupancy_grid.py    # Terminal 3
rosrun cs148_final_project dijkstra_planner.py  # Terminal 4
rosrun cs148_final_project path_follower.py     # Terminal 5
```

## Using the Program
Select the "2D Nav Goal" in the top left bar. Click on a point within the map to mark a goal position. You will see three lines form:
- Green --> A*
- Red --> Dijkstra
- Blue --> Best-First

In the event only 1 or 2 path show up, it may be due to the fact that some algorithms form the same path.