import argparse
import ast
import configparser
import os
import time

import numpy as np
import heapq
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity

from rclpy.qos import (
    ReliabilityPolicy,
    QoSProfile,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PIL import Image
from geometry_msgs.msg import Twist


np.set_printoptions(
    2, suppress=True, threshold=np.inf
)  # Print numpy arrays to specified d.p., suppress scientific notation (e.g. 1e-5), and do not truncate

set_logger_level("waypoint", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG

occupancy_grid_resolution = 0.2 # Sim (and grid array) resolution, in metres per cell
irl_resolution = occupancy_grid_resolution / 2 # The real maze is built at half the scale of the Gazebo maze -- same layout, 0.1m cells instead of 0.2m
max_translate_velocity = 1.4 # Overwritten in main() based on sim vs real-life; 0.3m/s cap for real life, please keep that in place

_PACKAGE_DIR = os.path.dirname(os.path.realpath(__file__))


# --- Coordinate conversion --------------------------------------------------
# A grid index (i, j) represents a CELL, not a point. That cell's world
# coordinate is its CENTER, e.g. cell [0, 0] is centred half a resolution-step
# away from the grid's origin corner, not exactly on it. This matches how the
# Gazebo world and the real maze are physically laid out (goal tape/markers
# sit in the middle of a cell, not on its boundary line).
def grid_to_world(i:int, j:int, origin:tuple, resolution:float=occupancy_grid_resolution) -> tuple:
    '''Convert grid index (i, j) to the world (x, y) coordinate of that cell's centre.'''
    return (origin[0] + (i + 0.5) * resolution, origin[1] + (j + 0.5) * resolution)

def world_to_grid(x:float, y:float, origin:tuple, resolution:float=occupancy_grid_resolution) -> tuple:
    '''Convert a world (x, y) coordinate to the grid index (i, j) of the cell containing it.'''
    return (int(np.floor((x - origin[0]) / resolution)), int(np.floor((y - origin[1]) / resolution)))


# --- Sim / real-life maze profiles ------------------------------------------
# The simulation maze and both real mazes are built to the SAME layout, so
# they all load the same occupancy grid array (ca2_sim_map.npy). Everything
# that differs between them -- where the grid's [0, 0] corner sits in the
# world frame, the cell resolution (the real maze is built at half scale),
# the goal points, and the speed cap -- lives in one of these profiles,
# picked by a single switch: run without --maze for simulation, or with
# --maze 0 / --maze 1 for a real maze. The two real mazes' placement and
# goal points live in optitrack_variables.config so they can be updated
# without touching this file.
MAP_FILE = "ca2_sim_map.npy"

sim_config = {
    "map_file": MAP_FILE,
    "origin": (-1.0, -5.0),
    "resolution": occupancy_grid_resolution,
    "goal_list": [(3.5, -3.5), (3.3, 0.3), (2.5, -3.5), (-0.3, -3.7)],
    "max_translate_velocity": 1.4,
}

def load_irl_config(maze_index:int) -> dict:
    '''Load a real-maze profile (origin + goal list) for maze 0 or maze 1 from optitrack_variables.config'''
    parser = configparser.ConfigParser()
    config_path = os.path.join(_PACKAGE_DIR, "optitrack_variables.config")
    parser.read(config_path)
    section = f"maze{maze_index}"
    if section not in parser:
        raise ValueError(f"No [{section}] section found in {config_path}")
    origin = (parser.getfloat(section, "origin_x"), parser.getfloat(section, "origin_y"))
    goal_list = list(ast.literal_eval(f"[{parser.get(section, 'goal_list')}]"))
    return {
        "map_file": MAP_FILE,
        "origin": origin,
        "resolution": irl_resolution, # Real maze is half the scale of the sim maze (0.1m cells, not 0.2m)
        "goal_list": goal_list,
        "max_translate_velocity": 0.3, # Please keep this in place; 0.3m/s is more than fast enough
    }


# --- Path planning + PID helpers ---------------------------------------------
WALL_PENALTY = 3.0 # Extra A* cost to enter a free cell touching a wall
NEAR_WALL_PENALTY = 1.0 # Extra A* cost to enter a free cell two cells away from a wall
NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)) # 8-connected moves


def astar(grid_array:np.array, start:tuple, goal:tuple) -> list:
    '''A* on the occupancy grid. Returns the list of cells from start to goal, or None if there is no path.

    Step cost is 1 for a straight move and sqrt(2) for a diagonal, plus a penalty for entering a cell near a wall,
    so the path keeps to the middle of corridors wherever there is room. Diagonal moves that would cut the corner
    of a wall are not allowed.
    '''
    is_free = lambda c: 0 <= c[0] < grid_array.shape[0] and 0 <= c[1] < grid_array.shape[1] and grid_array[c] <= 50
    if not (is_free(start) and is_free(goal)):
        return None

    walls = np.pad(grid_array > 50, 2, constant_values=True) # Outside the grid counts as wall
    rows, cols = grid_array.shape
    near_wall = lambda r: np.any([walls[2+di:2+di+rows, 2+dj:2+dj+cols] for di in range(-r, r+1) for dj in range(-r, r+1)], axis=0)
    penalty = np.where(near_wall(1), WALL_PENALTY, np.where(near_wall(2), NEAR_WALL_PENALTY, 0.0))

    def heuristic(cell): # Octile distance: never overestimates the real cost, so the path found is the cheapest
        di, dj = abs(cell[0]-goal[0]), abs(cell[1]-goal[1])
        return max(di, dj) + (np.sqrt(2) - 1) * min(di, dj)

    open_heap = [(heuristic(start), 0, start)] # (estimated total cost, tie-breaker, cell)
    cost = {start: 0.0}
    came_from = {start: None}
    pushes = 0
    while open_heap:
        cell = heapq.heappop(open_heap)[2]
        if cell == goal: # Walk back through came_from to get the path
            path = []
            while cell is not None:
                path.append(cell)
                cell = came_from[cell]
            return path[::-1]

        for di, dj in NEIGHBOURS:
            nxt = (cell[0]+di, cell[1]+dj)
            if not is_free(nxt) or (di and dj and not (is_free((cell[0]+di, cell[1])) and is_free((cell[0], cell[1]+dj)))):
                continue
            new_cost = cost[cell] + np.hypot(di, dj) + penalty[nxt]
            if new_cost < cost.get(nxt, np.inf):
                cost[nxt] = new_cost
                came_from[nxt] = cell
                pushes += 1
                heapq.heappush(open_heap, (new_cost + heuristic(nxt), pushes, nxt))
    return None


class PID():
    '''PID controller, stepped once per timer tick (dt seconds). The output is clipped to +/- out_limit, and the
    integral to +/- integral_limit so it can't wind up'''
    def __init__(self, kp:float, ki:float, kd:float, out_limit:float, integral_limit:float=np.inf, dt:float=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit, self.integral_limit, self.dt = out_limit, integral_limit, dt
        self.reset()

    def reset(self):
        self.integral, self.prev_error = 0.0, None

    def step(self, error:float) -> float:
        self.integral = np.clip(self.integral + error * self.dt, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / self.dt
        self.prev_error = error
        return float(np.clip(self.kp*error + self.ki*self.integral + self.kd*derivative, -self.out_limit, self.out_limit))


class WaypointNode(Node):
    '''Node to calculate path and move robot towards given goal_coordinates, using pose info from either gazebo odometer or optitrack'''
    def __init__(self, map_array:np.array, goal_list:list, is_simulation:bool=True, origin:tuple=(0.0, 0.0), resolution:float=occupancy_grid_resolution):
        super().__init__('waypoint')
        self.get_logger().info("Starting WaypointNode")

        self.is_simulation = is_simulation

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

            self.map_sub = self.create_subscription(
                PoseStamped,
                '/vrpn_mocap/bingda_003/pose',
                self.optitrack_callback,
                qos_profile
                )

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.goal_list = goal_list
        self.map_array = map_array
        self.origin = origin # World (x, y) coordinate of the grid's [0, 0] corner. Use with grid_to_world()/world_to_grid()
        self.resolution = resolution # Metres per grid cell for this run (0.2 sim, 0.1 real -- real maze is half scale). Use with grid_to_world()/world_to_grid()

        self.pose = None
        self.path = [] # Set this to your planned route (a list of grid-index tuples, in travel order) once you've computed it -- it'll automatically show up in the terminal map print
        self._last_printed_path = None

        # --- Path planning + PID state ---
        self.state = "PLAN" # PLAN (A* to the next goal) -> DRIVE (follow the path) -> ... -> STOP
        self.goal_idx = 0 # Index of the goal in goal_list being driven to
        self.waypoints = [] # World (x, y) points of the current path. Set by set_waypoints()
        self.current_waypoint_idx = 0 # Path segment the robot is currently on
        self.goal_reached = False
        self.speed = 0.0 # Forward speed sent last tick, so speed changes can be ramped
        self.stop_ticks = 0 # Ticks spent stopped at the end, before shutting down

        # Distances and speeds scale with the cell size, so the same numbers suit the sim (0.2m) and real (0.1m) maze
        self.GOAL_TOL = 0.5 * resolution # Goal counts as reached within this distance (0.1m in sim)
        self.LOOKAHEAD = 1.0 * resolution # Aim this far ahead along the path, so corners are rounded smoothly
        self.MAX_SPEED = min(2.0 * resolution, max_translate_velocity) # 0.4 m/s in sim
        self.MIN_SPEED = min(1.0 * resolution, max_translate_velocity) # The sim robot doesn't move at all below ~0.1 m/s
        self.MAX_ACCEL = 5.0 * resolution # m/s^2: most the forward speed may change by per second
        self.MAX_TURN = 2.0 * max_translate_velocity # move_2D's own turn limit
        self.MIN_TURN = 1.5 if is_simulation else 0.3 # rad/s. The sim robot doesn't turn at all below ~0.8 rad/s
        self.BODY_OFFSET = 0.074 if is_simulation else 0.0 # The sim pose is the rear axle; steer the body's centre, 0.074m ahead

        # PID controllers
        self.distance_pid = PID(1.0, 0.0, 0.0, out_limit=self.MAX_SPEED) # Path length left (m) -> forward speed (m/s)
        self.heading_pid = PID(3.0, 0.0, 0.2, out_limit=self.MAX_TURN) # Heading error (rad) -> turn rate (rad/s)
        self.cross_track_pid = PID(1.0, 2.0, 0.0, out_limit=0.75 * resolution, integral_limit=0.05) # Distance off the path (m) -> how far to shift the aim point back towards it (m)

    def print_map(self):
        '''Prints the occupancy grid to the terminal: walls, your current position ('S'), all goal points ('W'/'G'),
        and your planned route (self.path) if you've set one ('*'). Safe to call anytime pose is known; does nothing
        useful before then. Called automatically from timer_callback() whenever self.path changes.'''
        if self.pose is None:
            return
        shape = self.map_array.shape
        clip = lambda cell: (int(np.clip(cell[0], 0, shape[0]-1)), int(np.clip(cell[1], 0, shape[1]-1)))
        current_cell = clip(world_to_grid(self.pose[0], self.pose[1], self.origin, self.resolution))
        goal_cells = [clip(world_to_grid(gx, gy, self.origin, self.resolution)) for gx, gy in self.goal_list]
        grid = Grid(self.map_array, starting_position=current_cell, goal_position=goal_cells[-1])
        grid.print_grid_map(waypoints=goal_cells, path=self.path)

    def yaw_from_quaternion(self, q):
        '''Returns yaw angle (in rad) for orientation based on given quaternion input q'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.pose = np.array((x,y,heading))
        return self.pose

    def odometer_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def set_waypoints(self, waypoints:list):
        '''Set new waypoints when a goal has been reached'''
        self.goal_reached = False
        self.waypoints = waypoints
        self.current_waypoint_idx = 0

    def follow_path(self, position:np.array) -> tuple:
        '''Returns (carrot, remaining, cross_track, left) for the current path:
            carrot      : the point LOOKAHEAD further along the path than the closest point to the robot, which the robot aims at
            remaining   : path length left to the goal
            cross_track : distance from the path, positive when the robot is left of it
            left        : unit vector pointing left of the path
        Only the next few segments are searched for the closest point, so where the path doubles back the robot
        can't skip ahead onto a later part of it.'''
        points = np.array(self.waypoints, dtype=float)
        segments = np.diff(points, axis=0)
        lengths = np.hypot(segments[:, 0], segments[:, 1])
        along = np.concatenate(([0.0], np.cumsum(lengths))) # Path distance from the start to each waypoint

        best_distance, best_t = np.inf, 0.0
        for k in range(self.current_waypoint_idx, min(self.current_waypoint_idx + 4, len(segments))):
            t = np.clip(np.dot(position - points[k], segments[k]) / max(lengths[k]**2, 1e-9), 0.0, 1.0)
            distance = np.hypot(*(position - points[k] - t * segments[k]))
            if distance < best_distance:
                best_distance, best_t, self.current_waypoint_idx = distance, t, k
        k = self.current_waypoint_idx
        s = along[k] + best_t * lengths[k] # Path distance from the start to the closest point

        left = np.array((-segments[k, 1], segments[k, 0])) / max(lengths[k], 1e-9)
        cross_track = float(np.dot(position - points[k] - best_t * segments[k], left))
        carrot = np.array((np.interp(s + self.LOOKAHEAD, along, points[:, 0]), np.interp(s + self.LOOKAHEAD, along, points[:, 1])))
        return carrot, along[-1] - s, cross_track, left

    def drive(self, speed:float, turn:float):
        '''Ramp the forward speed towards speed by at most MAX_ACCEL per second, then send it with the turn rate.
        Like a car: forward speed and turning only, never sideways'''
        step = self.MAX_ACCEL * 0.05 # Timer period is 0.05s
        self.speed = float(np.clip(speed, self.speed - step, self.speed + step))
        self.move_2D(self.speed, 0.0, turn)

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            return # Does not run if no pose received from Odom or Optitrack
        self.get_logger().debug(f"Pose: {self.pose}")

        if self.path != self._last_printed_path: # Prints once immediately (map + start + goals), then again each time self.path changes
            self.print_map()
            self._last_printed_path = list(self.path)

        ###### INSERT CODE HERE ######
        heading = np.deg2rad(self.pose[2])
        position = self.pose[:2] + self.BODY_OFFSET * np.array((np.cos(heading), np.sin(heading))) # Middle of the robot's body

        if self.state == "PLAN": # A* from the robot's current cell to the next goal. Planned from the live pose, so drift doesn't carry over
            goal = self.goal_list[self.goal_idx]
            cells = astar(self.map_array, world_to_grid(*position, self.origin, self.resolution), world_to_grid(*goal, self.origin, self.resolution))
            if cells is None:
                self.get_logger().error(f"No path to goal {self.goal_idx+1} at {goal}")
                self.state = "STOP"
            else:
                self.path = self.path + cells # Keep every leg, so the terminal map shows the whole route
                # Path to follow: the robot's position -> centre of each cell in between -> exactly on the goal
                self.set_waypoints([tuple(position)] + [grid_to_world(i, j, self.origin, self.resolution) for i, j in cells[1:-1]] + [goal])
                self.heading_pid.reset()
                self.get_logger().info(f"Goal {self.goal_idx+1}/{len(self.goal_list)} at {goal}: {len(cells)} cells")
                self.state = "DRIVE"

        elif self.state == "DRIVE":
            if np.hypot(*(position - self.waypoints[-1])) < self.GOAL_TOL: # Reached the goal
                self.goal_reached = True
                self.get_logger().info(f"Reached goal {self.goal_idx+1} at {self.goal_list[self.goal_idx]}, pose {self.pose}")
                self.goal_idx += 1
                self.state = "PLAN" if self.goal_idx < len(self.goal_list) else "STOP"
            else:
                carrot, remaining, cross_track, left = self.follow_path(position)
                carrot = carrot + self.cross_track_pid.step(-cross_track) * left # Shift the aim point to pull the robot back onto the path
                to_carrot = carrot - position
                if np.hypot(*to_carrot) < 0.25 * self.resolution: # Right on top of the aim point (only happens at the goal)
                    heading_error = 0.0
                else:
                    heading_error = (np.rad2deg(np.arctan2(to_carrot[1], to_carrot[0])) - self.pose[2] + 180.0) % 360.0 - 180.0 # Degrees, -180 to 180

                # Turn: nothing inside a 3 deg deadband, otherwise the heading PID, but never weaker than MIN_TURN
                if abs(heading_error) < 3.0:
                    self.heading_pid.reset()
                    turn = 0.0
                else:
                    turn = np.sign(heading_error) * np.clip(abs(self.heading_pid.step(np.deg2rad(heading_error))), self.MIN_TURN, self.MAX_TURN)

                # Speed: set by the path length left; full speed within 15 deg of the aim point, slowing to a turn on the spot at 45 deg
                aim = np.clip((45.0 - abs(heading_error)) / 30.0, 0.0, 1.0)
                speed = np.clip(self.distance_pid.step(remaining), self.MIN_SPEED, self.MAX_SPEED)
                self.drive(max(speed * aim, self.MIN_SPEED) if aim > 0 else 0.0, turn)

        elif self.state == "STOP": # Ramp down to a standstill, hold it for a few ticks, then shut down
            self.drive(0.0, 0.0)
            if self.speed == 0.0:
                self.stop_ticks += 1
                if self.stop_ticks > 5:
                    self.get_logger().info("Finished, shutting down")
                    raise SystemExit
        ###### INSERT CODE HERE ######


class Grid():
    '''
    Grid class to use with occupancy grid. Contains the following functions:
        check_grid_validity : Uses flood fill to check if there's a valid path from start to goal position
        draw_grid_map : Creates a colour image of the grid, as well as waypoints and full solution path if given. Can show it and/or save it to a .png file
        print_grid_map : Prints an ASCII-art version of the same map (grid, waypoints, path) to the terminal
        animate_path : Prints the ASCII-art map frame by frame in the terminal, animating the robot moving along a given path

    __init__ input Args:
        grid_array : 2D numpy array representing the occupancy grid
        starting_position : tuple of starting indices within the numpy array
        goal_position : tuple of goal indices within the numpy array. Works with negative indices as well
    '''
    def __init__(self, grid_array:np.array=np.array([]), starting_position:tuple=(0,0), goal_position:tuple=(-1,-1)):
        self.grid = grid_array
        self.shape = self.grid.shape
        self.starting_position = starting_position

        # If goal position given with negative index, need to convert to +ve
        if goal_position[0] < 0:
            goal_x = self.shape[0] + goal_position[0]
        else:
            goal_x = goal_position[0]
        if goal_position[1] < 0:
            goal_y = self.shape[1] + goal_position[1]
        else:
            goal_y = goal_position[1]

        self.goal_position = (goal_x, goal_y)

    def check_grid_validity(self):
        '''Use flood fill to check if there's a viable path between start and goal positions'''
        grid = self.grid.copy()
        flood_stack = [(self.goal_position[0], self.goal_position[1])]
        while flood_stack:
            tile = flood_stack[0]
            del flood_stack[0]
            try:
                next_tile = (tile[0]+1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0]-1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]+1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]-1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
        if grid[self.starting_position] == 1: # Means the flood is able to reach starting position from the ending position
            return True
        else:
            return False

    def _colour_grid(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50) -> np.array:
        '''Builds the (H, W, 3) colour image array shared by draw_grid_map. Maze walls in blue, empty space in white, path taken in green and waypoints in red'''
        image_grid = np.ones((self.grid.shape[0],self.grid.shape[1],3), dtype=np.uint8)
        image_grid[self.grid <= obstacle_threshold] = (255,255,255)
        image_grid[self.grid > obstacle_threshold] = (0,0,255)

        for x, y in path:
            image_grid[x][y] = (0,255,0)

        for point in waypoints:
            image_grid[point] = (255,0,0)

        return image_grid

    def draw_grid_map(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50, save_path:str=None, show:bool=True):
        '''Creates an image of the maze and path taken. Maze walls in blue, empty space in white, path taken in green and waypoints in red

        Args:
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints. Will be represented in red, takes precedence over path
            path : list (or other iterable) of tuple coordinates representing all the grid indices forming the solution path. Will be represented in green
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2
            save_path : Optional file path (e.g. "path_result.png") to save the image to, in addition to/instead of showing it
            show : Whether to pop up the image in a viewer (default True). Set to False if you only want to save it
        '''
        image_grid = self._colour_grid(waypoints, path, obstacle_threshold)

        image_grid = np.flip(image_grid, axis=1)[::-1]
        img = Image.fromarray(image_grid, 'RGB')

        # Resize image
        base_width = 500
        wpercent = (base_width / float(img.size[0]))
        hsize = int((float(img.size[1]) * float(wpercent)))
        img = img.resize((base_width, hsize), Image.Resampling.NEAREST)

        if save_path:
            img.save(save_path)
            print(f"Saved map image to {save_path}")

        if show:
            img.show()

    def print_grid_map(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50):
        '''Prints an ASCII-art version of the maze to the terminal: '#' wall, '.' free, '*' solution path, 'W' waypoint, 'S' start, 'G' goal

        Args:
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints
            path : list (or other iterable) of tuple coordinates representing all the grid indices forming the solution path
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2
        '''
        chars = np.where(self.grid > obstacle_threshold, '#', '.').astype('<U1')

        for x, y in path:
            chars[x, y] = '*'
        for point in waypoints:
            chars[point] = 'W'
        chars[self.starting_position] = 'S'
        chars[self.goal_position] = 'G'

        display = np.flip(chars, axis=1)[::-1]
        print('\n'.join(''.join(row) for row in display))

    def animate_path(self, path:list, waypoints:list=(), obstacle_threshold:float=50, delay:float=0.2):
        '''Animates the robot moving along path, one cell at a time, by reprinting the ASCII-art map to the terminal.
        'R' marks the robot's current cell, '*' cells it has already passed through.

        Args:
            path : list of tuple grid indices forming the solution path, in travel order
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2
            delay : Seconds to pause between animation frames
        '''
        path = list(path)
        base_chars = np.where(self.grid > obstacle_threshold, '#', '.').astype('<U1')
        for point in waypoints:
            base_chars[point] = 'W'
        base_chars[self.goal_position] = 'G'

        for step in range(len(path)):
            frame = base_chars.copy()
            for x, y in path[:step]:
                frame[x, y] = '*'
            frame[path[step]] = 'R'

            display = np.flip(frame, axis=1)[::-1]
            print("\033c", end="")  # Clear terminal between frames
            print('\n'.join(''.join(row) for row in display))
            time.sleep(delay)


def main(args=None):
    global max_translate_velocity

    arg_parser = argparse.ArgumentParser(description="RB2301 CA2 path planning")
    arg_parser.add_argument(
        '--maze', type=int, choices=[0, 1], default=None,
        help="Which real-life maze to run on (0 or 1), configured in optitrack_variables.config. Omit this flag to run in Gazebo simulation."
    )
    cli_args, ros_args = arg_parser.parse_known_args(args=args)
    is_simulation = cli_args.maze is None # Remember: pass --maze 0 or --maze 1 to ca2.sh when testing on the real lab setup

    print("Starting path planning")
    rclpy.init(args=ros_args)

    if is_simulation:
        config = sim_config
    else:
        config = load_irl_config(cli_args.maze)
        print(f"Running on real maze {cli_args.maze}, origin={config['origin']}, resolution={config['resolution']}, goals={config['goal_list']}")

    max_translate_velocity = config["max_translate_velocity"]

    map_array = np.load(os.path.join(_PACKAGE_DIR, config["map_file"]), allow_pickle=True)
    waypoint = WaypointNode(map_array, config["goal_list"], is_simulation, config["origin"], config["resolution"])

    # Start spinning the waypoint node and only stop once SystemExit error is raised within the node callback
    try:
        rclpy.spin(waypoint)
    except SystemExit:
        print("Shutting down")

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()