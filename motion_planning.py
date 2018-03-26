import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import create_grid, a_star, heuristic, prune_path, find_start_goal
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global

from skimage.morphology import medial_axis
from skimage.util import invert

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        #----------------------------------------------------------------------------
        # HOME POSITION
        
        # TODO: read lat0, lon0 from colliders into floating point values
        # Open coliders.csv and read lat0 and lon0
        colliders_file = 'colliders.csv'

        with open(colliders_file) as f:
            latlon = f.readline().strip().split(',')
            lat0 = float(latlon[0].replace("lat0 ", ""))
            lon0 = float(latlon[1].replace("lon0 ", ""))
            print('lat: ', lat0)
            print('lon: ', lon0)

        # finished but latitude longitude are hardcoded for now: set home position to (lat0, lon0, 0)
        # I have an issue where float() truncates the last 0 and therefore provides a different value 
        #self.set_home_position(lon0, lat0, TARGET_ALTITUDE)
        self.set_home_position(-122.397450, 37.792480, TARGET_ALTITUDE)
        print('Home position is set to local position: ', self.local_position)
        # END HOME POSITION
        #----------------------------------------------------------------------------

        #----------------------------------------------------------------------------
        # CURRENT POSITION
        # finished: convert self.global_position to current local position using global_to_local()
        current_global_position = [self._longitude, self._latitude, self._altitude]
        print('current_global_position converted using global_to_local: {0}'.format(global_to_local(current_global_position,self.global_home)))
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))
        # END CURRENT POSITION
        #----------------------------------------------------------------------------

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        skeleton = medial_axis(invert(grid))
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # create_grid_and_edges uses Voronoi, it is slower than the previous create_grid() but it's suppose to create a safer path
        #grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        #skeleton = medial_axis(invert(grid))
        #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        #----------------------------------------------------------------------------
        # START POINT
        # Define starting point on the grid (this is just grid center)
        # start_position = (-north_offset, -east_offset)
        
        # finished: convert start position to current position rather than map center
        grid_start = (-north_offset + int(self.local_position[0]), -east_offset + int(self.local_position[1]))
        print('start: ', grid_start)
        # END START POINT
        #----------------------------------------------------------------------------
        
        #----------------------------------------------------------------------------
        # SET GOAL POINT
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        # finished: adapt to set goal as latitude / longitude position and convert
        # some lat lon that are working
        # first path
        goal_lat = float(37.795240)
        goal_lon = float(-122.393136)

        # second path
        #goal_lat = float(37.796874)
        #goal_lon = float(-122.399683)

        # third path
        #goal_lat = float(37.793155)
        #goal_lon = float(-122.402035)

        # fourth path and back to center
        #goal_lat = float(37.792480)
        #goal_lon = float(-122.397450)

        goal_position = global_to_local((goal_lon,goal_lat,0),self.global_home)
        print('goal position converted using global_to_local: {0}'.format(global_to_local((goal_lon,goal_lat,0),self.global_home)))

        grid_goal = (-north_offset + int(goal_position[0]), -east_offset + int(goal_position[1]))
        # END START POINT
        #----------------------------------------------------------------------------

        #----------------------------------------------------------------------------
        # RUN A* USING MEDIAN PATH AND COST OF SQRT(2)
        #Run A* to find a path from start to goal
        # finished: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)

        print('original start and goal: ',grid_start, grid_goal)
        print('calibrated start and goal on skeleton: ', skel_start, skel_goal)
        
        median_path, cost = a_star(invert(skeleton).astype(np.int), 
                            heuristic, 
                            tuple(skel_start), 
                            tuple(skel_goal))
        print('Median path and cost: ', len(median_path))

        pruned_path = prune_path(median_path)
        print('pruned path and cost: ', len(pruned_path))
        # END RUN A*
        #----------------------------------------------------------------------------

        # Convert path to waypoints
        #waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        print(waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
