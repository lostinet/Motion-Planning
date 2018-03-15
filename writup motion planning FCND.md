## Project: 3D Motion Planning Writeup


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`


In `motion_planning.py` a new phase of flight named **PLNNING** is inserted between **ARMING** and **TAKEOFF**, and function `plan_path()` is called while **armed**. 

we take `colliders.csv` as a input to generate waypoints for the quadrotor. function `create_grid()` and `a_star()`  are defined in the `planning_utils.py` then called by `plan_path()`. 

`create_grid()` transform the raw obstacle map data into grip block combining the altitude and safety margin information. by defining the grid starting point and the grid goal point, a-star algorithm comes in handy.

a star algorithm `a_star()` generate a path which will convert to waypoints. Actions define how the vehicle can move among the grid when given its current node position. and Cost is associated with each possible action. Heuristic is simply the linear distance between current position and goal position.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract `lat0` and `lon0` as floating point values and use the `self.set_home_position()` method to set global home. Explain briefly how you accomplished this in your code.

```
with open('colliders.csv') as csvfile:
    reader = csv.reader(csvfile)
    rows = [row for row in reader]
    lat0,lon0 = rows[1][0],rows[1][1]
```
```
self.set_home_position(float(lon0),float(lat0),0)
```
just simply read the csv file and specify the lat and lon values for home position. 


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Step 1: retrieve current global position 

```
longi = self._longitude
lati = self._latitude
alti = self._altitude
global_position = [longi,lati,alti]
```

Step 2: convert to current local position using `global_to_local()`

```
current_local_pos = global_to_local(global_position,self.global_home)
```


#### 3. Set grid start position from local position

simply add `current_local_pos` onto pre-defined offset. 

```
grid_start =(int(current_local_pos[0]+north_offset),int(current_local_pos[1]+east_offset))
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

adapt to set goal as latitude / longitude position and convert

```
goal_local_position = global_to_local([ -122.397934,37.794700,0],self.global_home)
grid_goal = (int(goal_local_position[0]+north_offset),int(goal_local_position[1]+east_offset))
```

some arbitrary nodes:

```
-122.397337,37.792570 -> Succeed
-122.397350,37.792570 -> Failed to find a route
-122.397934,37.794700 -> Crashed even succeed
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

Diagonal motion added into action list with a cost of sqrt(2)

```
NW = (-1, -1, math.sqrt(2))
SW = (1, -1, math.sqrt(2))
NE = (-1, 1, math.sqrt(2))
SE = (1, 1, math.sqrt(2))
```

if current position locates at the border,or both orthogonal sub-component directions blocked or the diagonal node is blocked. then remove cooresponding action towards that bearing.

```
if x-1<0 or y-1<0 or (grid[x-1,y]==1 and grid[x,y-1]==1) or grid[x-1,y-1]==1:
    valid_actions.remove(Action.NW)
if x-1<0 or y+1>m or (grid[x-1,y]==1 and grid[x,y+1]==1) or grid[x-1,y+1]==1:
    valid_actions.remove(Action.NE)
if x+1>n or y-1<0 or (grid[x+1,y]==1 and grid[x,y-1]==1) or grid[x+1,y-1]==1:
    valid_actions.remove(Action.SW)
if x+1>n or y+1>m or (grid[x+1,y]==1 and grid[x,y+1]==1) or grid[x+1,y+1]==1:
    valid_actions.remove(Action.SE)
```

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

collinerity was check to prune the path in planning_utils.py

```
def collinearity_check(p1, p2, p3, epsilon=1e-6):  
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])

        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
    
```

waypoints pruned before send to the simulator.

```
waypoints = [(p[0] - north_offset, p[1] - east_offset, TARGET_ALTITUDE+1) for p in path]
waypoints = prune_path(waypoints)
self.waypoints = waypoints
self.send_waypoints()
```

### Execute the flight
#### 1. Does it work?


I succeed to fly thru below nodes one by one, it works.

```
-122.397337,37.792570 
-122.397337,37.793800
-122.397337,37.795000 
-122.397337,37.796500 
```

but if I fly towards the 4th node directly i.e [-122.397337,37.796500], path found but the quadrotor is not responding in the simulator. 

**@reviewer: any suggestions to toubleshoot this issue?**

```
Found a path.
[(0, -1, 6), (26, -27, 6), (27, -27, 6), (37, -17, 6), (72, -17, 6), (144, -89, 6), (145, -89, 6), (147, -87, 6), (192, -87, 6), (202, -97, 6), (262, -97, 6), (282, -117, 6), (302, -117, 6), (312, -127, 6), (322, -127, 6), (326, -131, 6), (326, -163, 6), (337, -174, 6), (390, -174, 6), (397, -167, 6), (442, -167, 6), (443, -168, 6), (443, -169, 6), (444, -170, 6)]
Sending waypoints to simulator ...
Closing connection ...
```


### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.




