# RUBRIC Writeup

The writeup is an explanation of code changes and answers to Rubrik Questions to implement the FCND project, Motion Planning. 

## 1. Explain the starter code.
Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning.

### Answer:     
- To start with. There are few more imports of packages in the new motion_planning, especially the planning_utils is useful as it provides the a_star and heuristic function used in the Motion_planning as to plan a path

- Second the enumeration states are different, the motion planning provides one additional state, namely, PLANNING and all states have the value auto() instead of a default number. auto() means that an exact value is unimportant. 

The video "Phases of Flight" explains the phases (states) in a comprehensive way. I hope that the image is an reasonable attempt to explain the difference between the backyard-flyer and the motion_planner. The biggest difference can be found with the extra state, PLANNING. The ideas is that we don't have to fly in a hard coded box, but instead we can define a start and goal and the search function will provide flight plan (see code ==>  plan_path ) 
![planning phases](./misc/states/states.png)

## 2. Implementing your planning Algorithm
## Home Position
In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position())

### Answer: Home Position
- Read first line of the colliders,csv file and extract lat0, lon0 as floating point values. 

       `with open(colliders_file) as f:
            latlon = f.readline().strip().split(',')
            lat0 = float(latlon[0].strip().strip('lat0'))
            lon0 = float(latlon[1].strip().strip('lon0'))`
            
- set global home ==> `self.set_home_position(lat0, lon0, 0)`

## Retrieve current position
In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from 

`self._latitude` 
`self._longitude` 
`self._altitude`

Then use the utility function `global_to_local()` to convert to local position (using `self.global_home` as well, which you just set)

### Answer: Retrieve current position
Determine the local position relative to global home
`global_to Local()` is a function you can find in frame_utils.py (udacidrone)

1. get easting and northing of global home
        `(east_home, north_home, _, _) = utm.from_latlon(self.global_home[1],self.global_home[0])`
        
2. get easting and northing of global position
        `(east, north, _, _) = utm.from_latlon(global_position[1],global_position[0])`
        
3. create a local position from global and home positions
        `local_position = numpy.array([north - north_home, east - east_home, -global_position[2]])`
        
## change start point
In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

### Answer: start point
The previous rubrik retrieved the current position. Therefore the north start point = current_local_position - north_offset, 
and the east start point = current_local_position - east_offset
    `north_start = current_local_position[0] - north_offset`
    `east_start = current_local_position[1] - east_offset`
    `grid_start = (north_offset, east_offset)`     


## Set goal position
In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

### Answer: Set goal Position
The goal can be any lat, lon within the map and have it rendered to a goal location on the grid.

1. set latitude,   `goal_lat = float(37.796874)`
2. set longitude,  `goal_lon = float(-122.399683)`
3. set the altitide to the altitude of the global home with the defined latitude and longitude
                   `goal = [goal_lon, goal_lat, self.global_home[2]]`
4. convert the global goal position to the local goal position by calling the function global_to_local()
        `goal_position = global_to_local(goal, self.global_home)`
5. and finally set the grid_goal with the offset included
        `grid_goal = (int(goal_position[0] + north_offset), int(goal_position[1] + east_offset))`
