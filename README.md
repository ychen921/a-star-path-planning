# a-star-path-planning
Path planning using A* algorithm and implementation on turtlebot waffle in Gazebo simulation.

## Author
- Ji Liu - 112960186 (jiliu@umd.edu)
- Yi-Chung Chen - 119218990 (ychen921@umd.edu)

## Dependencies
- numpy
- matplotlib
- multiprocessing
- heapq
- argparse

## Usage
### Part 1 - A* Visualization


### Part 2 - Gazebo Simulation
Source ROS
```
source install/setup.bash
```

Build the workspace
```
cd project3_ws
colcon build --packages-select turtlebot3_project3
```

Launch Environment
```
ros2 launch turtlebot3_project3 competition_world.launch.py
```

To run the ROS node, you can use the [vel_publisher.py](/scripts/astar_sim.py) for the following:

```
ros2 run turtlebot3_project3 vel_publisher.py
```
To adjust the goal point, use `--GoalNode` to set the point. Here is an example:
```
ros2 run turtlebot3_project3 vel_publisher.py ---GoalNode 5750 500
```
