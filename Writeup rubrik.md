### RUBRIC Writeup

The writeup is an explanation of code changes and answers to Rubrik Questions to implement the FCND project, Motion Planning. 

### 1. Very brief explanation of starter code.
_Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning._

##### Answer:     
- To start with, a few more packages are imported in the new motion_planning, especially the planning_utils is useful as it provides the a_star and heuristic function used in the Motion_planning as to plan a path

- Second the enumeration states are different, the motion planning provides one additional state, namely, PLANNING and all states have the value auto() instead of a default number. auto() means that an exact value is unimportant. 

The video "Phases of Flight" is an comprehensoive explanation of different flight phases (states). The image below is (hopefully) a reasonable attempt to show the difference between the diferent states used in the backyard-flyer and the motion_planner. The biggest difference is found with the extra state, "PLANNING". The ideas is that we don't fly in a hard coded box, but instead we can define a "GOAL" and the search function will provide a flight plan (see code ==>  plan_path ) 

![planning phases](./misc/states/states.png)

### 2. Implementing your planning Algorithm
#### Home Position
_In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position())_

##### Answer: Home Position
- Read first line of the colliders,csv file and extract lat0, lon0 as floating point values. 

       `with open(colliders_file) as f:
            latlon = f.readline().strip().split(',')
            lat0 = float(latlon[0].strip().strip('lat0'))
            lon0 = float(latlon[1].strip().strip('lon0'))`
            
- set global home ==> `self.set_home_position(lat0, lon0, 0)`

Although the first line is split in a LAT LON, the float seems to truncate the last zero. Meaning `float(-122.397450)` is returned as `-122.39745`. in order to avoid this the latlon is hardcoded. I will investigate after the project submission how I can better extract the lat lon and convert it to a float where the trailing `0` is not truncated.


#### Retrieve current position
_In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from_

`self._latitude` 
`self._longitude` 
`self._altitude`

Then use the utility function `global_to_local()` to convert to local position (using `self.global_home` as well, which you just set)_

##### Answer: Retrieve current position
Determine the local position relative to global home
`global_to Local()` is a function you can find in frame_utils.py (udacidrone)

1. get easting and northing of global home
        `(east_home, north_home, _, _) = utm.from_latlon(self.global_home[1],self.global_home[0])`
        
2. get easting and northing of global position
        `(east, north, _, _) = utm.from_latlon(global_position[1],global_position[0])`
        
3. create a local position from global and home positions
        `local_position = numpy.array([north - north_home, east - east_home, -global_position[2]])`

the function glocal_to_local() requires two parameters. The first one is the global_position and the second is the global_home. The geodetic position is retrieved from `self._latitude, self._longitude and self._altitude` (self.global_home can be used instead)

        
#### change start point
_In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position._

##### Answer: start point
Two additional variables, the __*north_offset*__ and __*east_offset*__ are defined upon running the function `grid_creation`. Both are required to specifiy the start position on the grid. the start position is basically -offset + local_position
        
`grid_start = (-north_offset + int(self.local_position[0]), -east_offset + int(self.local_position[1]))`


#### Set goal position
_In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)_

##### Answer: Set goal Position
The goal can be any __lat, lon__ rendered (using global_to_local()) to a goal location on the grid. 

1. set latitude,    `goal_lat = float(37.795240)`
2. set longitude,   `goal_lon = float(-122.393136)`
3. use global_to_local to convert the latlon into local NED
                    `goal_position = global_to_local((goal_lon,goal_lat,0),self.global_home)`
4. set the grid_goal by adding the offset values
                    `grid_goal = (-north_offset + int(goal_position[0]), -east_offset + int(goal_position[1]))`

##### first path from center to point 1
|                         | from start to point1  |
|------------------------ | :--------------------:|
| Goal Lattitude          | lat: 37.795240        |
| Goal Longitude.         | lon: -122.393136      |
| Start (local)           | [316 445]             |
| Goal (local)            | [626 820]             |
| waypoints median path.  | 865                   |
| waypoints pruned path.  | 149                   |

![first path from center to point 1](./misc/paths/path_center_pos1.png)

##### second path from point 1 to point 2
|                         | from point1 to point2 |
|------------------------ | :--------------------:|
| Goal Lattitude          | lat: 37.796874        |
| Goal Longitude.         | lon: -122.399683      |
| Start (local)           | [626 820]             |
| Goal (local)            | [820 250]             |
| waypoints median path.  | 757                   |
| waypoints pruned path.  | 144                   |

![first path from 1 to point 2](./misc/paths/path_pos1_pos2.png)

##### third path from point 2 to point 3
|                         | from point2 to point3 |
|------------------------ | :--------------------:|
| Goal Lattitude          | lat: 37.793155        |
| Goal Longitude.         | lon: -122.402035     |
| Start (local)           | [820 250]             |
| Goal (local)            | [388 35]             |
| waypoints median path.  | 654                   |
| waypoints pruned path.  | 125                   |

![first path from point 2 to point 3](./misc/paths/path_pos2_pos3.png)

##### third path from point 3 back to center
|                         | from point2 to point3 |
|------------------------ | :--------------------:|
| Goal Lattitude          | lat: 37.792480        |
| Goal Longitude.         | lon: -122.397450     |
| Start (local)           | [388 35]             |
| Goal (local)            | [316 445]             |
| waypoints median path.  | 514                   |
| waypoints pruned path.  | 89                   |

![first path from point 3 back to center](./misc/paths/path_pos2_center.png)
#### Search Algorithm A*


#### Cull waypoints

