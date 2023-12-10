# sailbot_pathfinding

ROS2 Humble package containing WPI Sailbot's pathfinding system

# overview

This package contains a single node (pathfinder_node.cpp), which runs two services (`set_map` and `get_path`). the `set_map` service receives an [`OccupancyGrid`](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html), and creates an internal map which future `get_path` requests will use. The `get_path` service receives two [`Point`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Point.html) messages (start and goal), as well as a `float64` message (wind angle, in degrees above the positive X axis), and returns a [`Path`](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) containing a sailable path between `start` and `goal` (empty if no path was found).

The pathfinding system currently runs in three stages- first, a raycast is performed to check if the boat can move from `start` to `goal` directly. If such a maneuver would sail the boat upwind (in the no-sail zone), we fall back to a single-tack strategy through intermediate point C, with two raycasts. If, in either previous stage, the path is blocked because of an obstacle on the map, we then fall back to true pathfinding. Each stage is explained in more detail below.

# Wind Check
Before pathfinding, we check if direct movement between `start` and `goal` would sail us into the "no-sail" zone (too far upwind). If so, we skip directly to the second stage of pathfinding.

# Linear Raycast
The first pathfinding stage, linear raycast, uses a [modified form](http://eugen.dedu.free.fr/projects/bresenham/) of the [Bresenham line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm). This is used to raycast over the grid from `start` to `goal`, failing if an obstacle is encountered. If successful, the `start` and `goal` positions are returned directly, as a `Path`. If unsuccessful, we fall back to stage 3.

# Single-Tack
The second pathfinding stage, Single-Tack, calculates two intermediate points `C1` and `C2`, such that moving directly from `start` to `CX` to `goal` would sail the boat along the boundaries of the no-sail zone. A raycast is then performed along each section of both possible paths, and the first un-blocked path found is returned. If both paths are blocked, we fall back to stage 3.

# True Pathfinding
The third pathfinding stage uses a real pathfinding algorithm to find a sailable path between `start` and `goal`. This stage currently uses A*, though other strategies have been investigated (see [here](https://github.com/wpisailbot/pathfinding_playground)). First, the base map data (provided through the `set_map` service) is rotated such that the provided wind vector is axis-aligned with the grid. Then, an A* algorithm is run, using a 7-neighbor configuration (where the one removed neighbor ensures paths will never be generated which require sailing directly upwind). The algorithm penalizes turns, resulting in a path that is generally near-optimal, and which contains as few turns as possible. The result of these two restrictions (upwind restriction provided by the map rotation and 7-neighbor configuration, and turning restriction provided by turn penalties) is that the generated paths are generally sailable. Since boat dynamics are not actually calculated, however, particularly complex pathing situations could result in paths that the boat cannot feasibly sail. This has been deemed acceptable, since such complex situations are also highly unlikely to be encountered given the expected operational environments of WPI's sailbot.