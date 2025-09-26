## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

# Required Steps:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone's start location corresponds to [0, 0, 0, 0].

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

After analyzing the provided starter code, **motion_planning.py** was found to be a modified version of the backyard flyer that incorporates path planning capabilities. The key differences observed include:

- **Added PLANNING state**: A new state in the flight state machine specifically designed for path planning operations
- **Enhanced plan_path() method**: This method handles obstacle loading, grid creation, A* search, and waypoint generation - much more sophisticated than the simple square pattern in backyard flyer
- **Dynamic waypoint generation**: Unlike the hardcoded square pattern from the backyard flyer project, waypoints are generated based on A* pathfinding results
- **Integration with planning_utils**: The script leverages external planning utilities for grid creation, search algorithms, and path optimization

The **planning_utils.py** module contains the core planning algorithms and data structures:

- **create_grid()**: Converts obstacle data from colliders.csv into a 2D occupancy grid representation
- **Action enum**: Defines possible movement directions (originally 4-directional: North, South, East, West)
- **valid_actions()**: Determines which movements are valid from any given grid position based on obstacles and boundaries
- **a_star()**: Implements the A* search algorithm to find optimal paths between start and goal positions
- **heuristic()**: Uses Euclidean distance as the heuristic function for A* search

When the starter code was first executed, it produced a jerky northeast path of approximately 10m before landing. The implementation was hardcoded to start from map center and move to a fixed offset position, which limited its practical usefulness.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

To make the planner more flexible, the global home position needed to be read from the colliders.csv file instead of using a hardcoded value. This was implemented as follows:

**Implementation (motion_planning.py:122-130):**
```python
# Read lat0, lon0 from colliders into floating point values
with open('colliders.csv') as f:
    first_line = f.readline().strip()
lat0_str, lon0_str = first_line.split(', ')
lat0 = float(lat0_str.split(' ')[1])
lon0 = float(lon0_str.split(' ')[1])

# Set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
```

The first line of colliders.csv was found to contain the map's origin coordinates in the format "lat0 37.792480, lon0 -122.397450". The code parses these values by splitting the string and extracting the numeric values, then uses the `self.set_home_position()` method to establish the global home position. This ensures the drone's coordinate system is properly aligned with the map data, which is crucial for accurate planning.

#### 2. Set your current local position

The drone's current position needed to be determined in the local coordinate system relative to the global home that was just set. The approach was:

**Implementation (motion_planning.py:132-136):**
```python
# Retrieve current global position
current_global_position = [self._longitude, self._latitude, self._altitude]

# Convert to current local position using global_to_local()
current_local_position = global_to_local(current_global_position, self.global_home)
```

The drone's current global position is retrieved using the internal `_longitude`, `_latitude`, and `_altitude` properties, then converted to the local coordinate system using the provided `global_to_local()` utility function. This conversion is essential for relating the drone's real-world position to the grid-based planning space used for pathfinding.

#### 3. Set grid start position from local position

The hardcoded map center start position was replaced with the drone's actual current position to make the planner more practical:

**Implementation (motion_planning.py:147-148):**
```python
# Convert start position to current position rather than map center
grid_start = (int(current_local_position[0] - north_offset), int(current_local_position[1] - east_offset))
```

Instead of using the original hardcoded map center `(-north_offset, -east_offset)`, the start position is calculated from the drone's actual current local position. The north and east offsets are subtracted to convert from local coordinates to grid indices. This change provides the flexibility to start planning from any position where the drone is currently located, making it much more useful for real-world applications.

#### 4. Set grid goal position from geodetic coords

Goal selection was made more flexible by accepting real-world latitude/longitude coordinates instead of hardcoded grid positions:

**Implementation (motion_planning.py:151-157):**
```python
# Adapt to set goal as latitude / longitude position and convert
# Example goal position (you can modify these coordinates)
goal_lon = -122.396428
goal_lat = 37.793837
goal_global_position = [goal_lon, goal_lat, 0]
goal_local_position = global_to_local(goal_global_position, self.global_home)
grid_goal = (int(goal_local_position[0] - north_offset), int(goal_local_position[1] - east_offset))
```

The goal position is specified using latitude/longitude coordinates rather than hardcoded grid offsets. The code converts these geodetic coordinates to local coordinates using `global_to_local()`, then transforms them to grid coordinates. This approach allows for flexible goal setting anywhere within the mapped environment by simply changing the lat/lon values, making the system much more practical for mission planning.

#### 5. Modify A* to include diagonal motion

The original 4-directional movement (North, South, East, West) was recognized as quite limiting and would result in jagged, inefficient paths. To improve this, diagonal movement capabilities were added:

**Enhanced Actions (planning_utils.py:58-61):**
```python
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```

**Enhanced valid_actions Function (planning_utils.py:92-100):**
```python
# Check diagonal movements
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```

Four diagonal movement actions were added, each with a cost of √2 (approximately 1.414) to accurately represent the longer distance traveled diagonally. The `valid_actions()` function was also updated to check boundary conditions and obstacle collisions for diagonal movements. This enhancement allows for much more natural and efficient flight paths compared to the original 4-directional movement, which was quite restrictive.

#### 6. Cull waypoints

A* was observed to often generate many unnecessary waypoints, especially when traveling in straight lines. To optimize this, a waypoint culling system was implemented using collinearity testing:

**Collinearity Implementation (planning_utils.py:166-189):**
```python
def collinearity_check(p1, p2, p3, epsilon=1e-6):
    """
    Check if three points are collinear using the determinant method.
    """
    m = np.concatenate((point(p1), point(p2), point(p3)), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    """
    Prune waypoints from a path using collinearity test.
    """
    if len(path) < 3:
        return path

    pruned_path = [path[0]]  # Always include start point

    for i in range(1, len(path) - 1):
        if not collinearity_check(pruned_path[-1], path[i], path[i + 1]):
            pruned_path.append(path[i])

    pruned_path.append(path[-1])  # Always include end point
    return pruned_path
```

**Usage (motion_planning.py:164-167):**
```python
# Prune path to minimize number of waypoints
original_path_length = len(path)
path = prune_path(path)
print(f"Original path length: {original_path_length}, Pruned path length: {len(path)}")
```

The waypoint culling implementation uses a collinearity test based on the determinant method. The `collinearity_check()` function determines if three consecutive points are collinear within a small epsilon tolerance. The `prune_path()` function iterates through the path, keeping only waypoints that represent significant direction changes. This optimization achieved remarkable results - waypoints were reduced from 163 to 20 (87% reduction) in the test execution while maintaining path accuracy!

### Execute the flight

#### 1. Does it work?

**Yes, it works exceptionally well!** The implementation performed very effectively.

**Execution Results:**
```
starting connection
arming transition
Searching for a path ...
global home [-122.39745   37.79248    0.     ], position [-122.3974497   37.7924799    0.211    ], local position [-0.0005739   0.01782577 -0.21197736]
North offset = -316, east offset = -445
Local Start and Goal:  (315, 445) (467, 534)
Found a path.
Original path length: 163, Pruned path length: 20
Sending waypoints to simulator ...
takeoff transition
[waypoint transitions showing 20 optimized waypoints]
landing transition
disarm transition
manual transition
```

**Key Performance Metrics:**
- **Path Planning**: Successfully found a path from start (315, 445) to goal (467, 534)
- **Waypoint Optimization**: Achieved an 87% reduction in waypoints (163 → 20)
- **Flight Execution**: The drone smoothly navigated through all 20 waypoints
- **Mission Completion**: Successful takeoff, navigation, and landing was observed
