# dynamic_obstacle_avoidance
This repository has two packages: A* Search global path planner and Dynamic Window Approach (DWA) local path planner.

Detailed Explanation at: https://juhyungsprojects.blogspot.com/2024/04/dynamic-window-approach-for-local-path.html

# A* Search Global Path Planner:
The A* Search global path planner finds the shortest trajectory from the robot's current pose to the goal pose using the static global costmap and the A* Search algorithm. The algorithm is a modified Breath-First Search algorithm where a priority queue is used instead of a regular queue. The priority queue sorts the inserted nodes based on their heuristics.The heuristics in this case is the sum of three components:
- Distance between the current node and the goal node
- Distance between the current node and the start node
- Occupancy value of the current node

Every time a node is inserted into the priority queue, the node with the lowest priority comes at the top and is therefore popped first. Intuitively, this means that nodes that are both closer to the goal and obstacle-free are searched first when looking for the shortest trajectory.

![](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExZ3pqZ3JsaDBlbjNwbXl6NDBkZmIyeHJkZTBvenhqMTh3MGxld28wZCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/z6Aq2UxRt1LF82msIE/giphy.gif)

# DWA Local Path Planner
The DWA local path planner samples a bunch of local trajectories (short trajectories) that the robot can follow. The sampled trajectories are scored by multiple critics and the trajectory with the highest score is chosen. There are 3 critics implemented in this repository: GlobalPathAlignCritic, HomingCritic, ObstacleProximityCritic.

GlobalPathAlignCritic:
- Penalizes trajectories that do not align with the global trajectory.

HomingCritic:
- If the goal pose is inside the local costmap, score local trajectories based on distance between goal pose and last pose of local trajectory

ObstacleProximityCritic:
- Penalizes trajectories that too are close to obstacles

![](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExdTJha2wwdGZueWlkMjE5YWdiM2RmdzRzYWNmbmx3c2JmZDRjZGR0aiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/A7IfpXWOYjAkB1DGcB/giphy-downsized-large.gif)
