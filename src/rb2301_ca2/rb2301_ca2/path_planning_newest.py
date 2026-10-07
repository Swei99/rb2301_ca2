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
    #"goal_list": [(3.5, -3.5), (3.3, 0.3), (2.5, -3.5), (-0.3, -3.7)],
    "goal_list": [(-0.4, -3.8), (0.0, 0.0), (2.4, -3.6), (3.2, 0.2)],
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


# --- Path planning ----------------------------------------------------------
# Plain A* over the occupancy grid. The only addition to textbook A* is a small extra cost for entering a
# free cell near a wall: free cells right beside a wall have their centre only ~0.1m from the real wall,
# which is less than half the robot's width, so the route is pushed further out wherever the corridor is
# wide enough to allow it. A lighter second ring keeps it off walls in open space too, which matters
# because the path follower rounds corners slightly rather than driving through every cell centre.
OBSTACLE_THRESHOLD = 50 # Cell values above this are walls. Matches the Grid class's default
WALL_PENALTY = 3.0 # Extra cost for a free cell touching a wall; worth a ~3 cell detour to avoid one
NEAR_WALL_PENALTY = 1.0 # Extra cost for a free cell two cells from a wall

NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def is_free(grid_array:np.array, cell:tuple) -> bool:
    '''Whether a cell is inside the grid and not a wall'''
    return (0 <= cell[0] < grid_array.shape[0] and 0 <= cell[1] < grid_array.shape[1]
            and grid_array[cell] <= OBSTACLE_THRESHOLD)


def near_wall_mask(grid_array:np.array, radius:int=1) -> np.array:
    '''True for every cell within radius cells (8-connected) of a wall. Outside the grid counts as wall'''
    walls = np.pad(grid_array > OBSTACLE_THRESHOLD, radius, constant_values=True)
    rows, cols = grid_array.shape
    mask = np.zeros((rows, cols), dtype=bool)
    for di in range(2*radius + 1):
        for dj in range(2*radius + 1):
            mask |= walls[di:di+rows, dj:dj+cols]
    return mask


def plan_astar(grid_array:np.array, start:tuple, goal:tuple) -> list:
    '''A* over the 8-connected grid. Returns the cells from start to goal inclusive, or None if unreachable.

    Step cost is 1 for a straight move, sqrt(2) for a diagonal, plus WALL_PENALTY when the cell entered touches
    a wall (NEAR_WALL_PENALTY when it is one cell further out). The octile-distance heuristic never overestimates
    that, so the path is optimal for this cost.
    Diagonal moves between two walls are not allowed, since the robot is wider than a point.
    '''
    if not is_free(grid_array, start) or not is_free(grid_array, goal):
        return None
    touching = near_wall_mask(grid_array, 1)
    extra_cost = WALL_PENALTY * touching + NEAR_WALL_PENALTY * (near_wall_mask(grid_array, 2) & ~touching)

    def heuristic(cell):
        di, dj = abs(cell[0]-goal[0]), abs(cell[1]-goal[1])
        return max(di, dj) + (np.sqrt(2) - 1) * min(di, dj)

    open_heap = [(heuristic(start), 0, start)]
    cost_so_far = {start: 0.0}
    came_from = {}
    counter = 0 # Tie-breaker so heapq never has to compare two cells
    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for di, dj in NEIGHBOURS:
            neighbour = (current[0]+di, current[1]+dj)
            if not is_free(grid_array, neighbour):
                continue
            if di and dj and not (is_free(grid_array, (current[0]+di, current[1])) and is_free(grid_array, (current[0], current[1]+dj))):
                continue # Would cut the corner of a wall
            new_cost = cost_so_far[current] + (np.sqrt(2) if di and dj else 1.0) + extra_cost[neighbour]
            if new_cost < cost_so_far.get(neighbour, np.inf):
                cost_so_far[neighbour] = new_cost
                came_from[neighbour] = current
                counter += 1
                heapq.heappush(open_heap, (new_cost + heuristic(neighbour), counter, neighbour))
    return None


# --- Control ----------------------------------------------------------------
def wrap_deg(angle:float) -> float:
    '''Normalise an angle in degrees into the -180 to 180 range that self.pose reports headings in'''
    return (angle + 180.0) % 360.0 - 180.0


class PID():
    '''Plain PID controller, stepped once per control tick. Output is clipped to +/- out_limit, and the
    accumulated integral to +/- integral_limit so it cannot wind up'''
    def __init__(self, kp:float, ki:float, kd:float, dt:float, out_limit:float, integral_limit:float=np.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.out_limit = out_limit
        self.integral_limit = integral_limit
        self.reset()

    def reset(self):
        '''Forget the accumulated integral and the last error. Call whenever the target changes'''
        self.integral = 0.0
        self.previous_error = None

    def step(self, error:float) -> float:
        '''Advance the controller by one tick and return the clipped output'''
        self.integral = float(np.clip(self.integral + error * self.dt, -self.integral_limit, self.integral_limit))
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / self.dt
        self.previous_error = error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        return float(np.clip(output, -self.out_limit, self.out_limit))


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

        # --- Path planning + control state ---
        self.DT = 0.05 # Controller period in seconds, matching the timer above
        self.state = "PLAN" # PLAN (route to the next goal) -> DRIVE (follow it) -> ... -> STOP
        self.goal_idx = 0 # Which goal in goal_list we are heading for
        self.waypoints = [] # World (x, y) points of the current leg's path, as one polyline. Filled in by set_waypoints()
        self.current_waypoint_idx = 0 # Segment of that polyline the robot is currently alongside
        self.goal_reached = False
        self.stop_ticks = 0

        # Tolerances and speeds scale with the cell size, so the same code suits the 0.2m sim and 0.1m real maze
        self.WAYPOINT_TOL = 0.5 * self.resolution # "Reached" radius for a waypoint (0.1m in sim)
        self.MAX_SPEED = min(2.0 * self.resolution, max_translate_velocity) # 0.4 m/s in sim
        # The simulated robot slides on fixed wheels, so it does not move at all below ~0.1 m/s. Never command
        # less than this while a waypoint is still ahead, or it stalls just short of it.
        self.MIN_SPEED = min(1.0 * self.resolution, max_translate_velocity) # 0.2 m/s in sim

        # The simulated robot's pose is its rear axle, but its body reaches 0.18m forward of that and 0.03m behind.
        # Steering the middle of the body along the path, not the rear axle, is what keeps its nose off the walls.
        self.BODY_OFFSET = 0.074 if self.is_simulation else 0.0 # m forward of the pose to the body's centre

        # Smooth driving: the robot chases a point LOOKAHEAD ahead of it along the path instead of stepping from
        # cell to cell, and its speed is ramped rather than jumped. Stepping cell to cell made it brake and surge
        # at every single cell.
        self.LOOKAHEAD = 1.0 * self.resolution # 0.2m in sim: rounds corners smoothly; 0.4m was long enough to cut one into a wall
        self.MAX_ACCEL = 5.0 * self.resolution # 1.0 m/s^2 in sim: most the forward speed may change by per second
        self.cmd_speed = 0.0 # Forward speed sent last tick, which the ramp starts from

        # Car-like steering: forward/backward speed and turning only, never sideways. The robot turns its nose
        # towards the carrot and drives forward, slowing the further off-aim it is and turning on the spot
        # when it is badly off (e.g. at the start of a leg that doubles back).
        self.FULL_SPEED_DEG = 15.0 # Heading error below which it drives at full speed
        self.TURN_ONLY_DEG = 45.0 # Heading error above which it stops and turns on the spot
        self.HEADING_DEADBAND_DEG = 3.0 # Heading error small enough to leave alone
        # The simulated robot slides on fixed wheels and will not rotate at all for turn commands below ~0.8 rad/s,
        # so a small correction has to be made as a short, firm turn rather than a gentle one.
        self.MIN_TURN = 1.5 if self.is_simulation else 0.3 # rad/s
        self.MAX_TURN = 2.0 * max_translate_velocity # move_2D's own turn limit
        # Heading PID: heading error (rad) -> turn rate (rad/s)
        self.heading_pid = PID(3.0, 0.0, 0.2, self.DT, out_limit=self.MAX_TURN)
        # Cross-track PID: sideways distance from the path (m) -> how far to slide the carrot sideways (m). The
        # simulated robot's friction acts along the world x and y axes, so a small heading error does not change
        # where it actually goes: it carries on dead along an axis, holding whatever offset it came out of the
        # last corner with (5cm was measured). Aiming at the carrot alone never sees enough error to fix that;
        # the integral keeps pushing the aim point further across until the heading is off by enough to break
        # it free, then lets go as the robot comes back onto the line.
        self.cross_track_pid = PID(1.0, 2.0, 0.0, self.DT, out_limit=0.75 * self.resolution, integral_limit=0.05)

        # Distance PID: path length left to the goal (m) -> speed (m/s). Cruises at MAX_SPEED until the last
        # ~MAX_SPEED/kp metres of the leg, then eases down into the goal.
        self.distance_pid = PID(1.0, 0.0, 0.0, self.DT, out_limit=self.MAX_SPEED)

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
        self.waypoints = [np.array(point, dtype=float) for point in waypoints]
        self.current_waypoint_idx = 0

    def body_centre(self) -> tuple:
        '''World (x, y) of the middle of the robot's body, which is what gets steered along the path'''
        heading = np.deg2rad(self.pose[2])
        return (self.pose[0] + self.BODY_OFFSET * np.cos(heading), self.pose[1] + self.BODY_OFFSET * np.sin(heading))

    def plan_leg(self) -> bool:
        '''A* from the robot's current cell to the next goal's cell. The path becomes one polyline to follow:
        from where the robot is now, through the centre of every cell, ending exactly on the goal. Planning from
        the live pose at the start of each leg stops any drift from the last leg carrying over.'''
        goal_x, goal_y = self.goal_list[self.goal_idx]
        start_cell = world_to_grid(*self.body_centre(), self.origin, self.resolution)
        goal_cell = world_to_grid(goal_x, goal_y, self.origin, self.resolution)

        cells = plan_astar(self.map_array, start_cell, goal_cell)
        if cells is None:
            self.get_logger().error(f"No path from {start_cell} to goal {self.goal_idx+1} at {goal_cell}")
            return False

        self.path = self.path + (cells[1:] if self.path else cells) # Accumulate every leg so the terminal map shows the whole route
        centres = [grid_to_world(i, j, self.origin, self.resolution) for i, j in cells[1:-1]]
        self.set_waypoints([self.body_centre()] + centres + [(goal_x, goal_y)])
        self.distance_pid.reset()
        self.heading_pid.reset()
        self.get_logger().info(f"Goal {self.goal_idx+1}/{len(self.goal_list)} at ({goal_x}, {goal_y}): {len(cells)} cells")
        return True

    def follow_path(self, position:np.array) -> tuple:
        '''Where to aim and how far off the path the robot is: returns (carrot point, remaining path length to the
        goal, cross-track error, path direction). Cross-track error is positive when the robot is left of the path.

        Finds the closest point on the leg's polyline to the robot, then walks LOOKAHEAD further along it. Only
        the next few segments are searched, so where the route doubles back past itself the robot cannot skip
        ahead onto a later part of it.
        '''
        points = self.waypoints
        best = (np.inf, self.current_waypoint_idx, points[self.current_waypoint_idx])
        for k in range(self.current_waypoint_idx, min(self.current_waypoint_idx + 4, len(points) - 1)):
            segment = points[k+1] - points[k]
            t = np.clip(np.dot(position - points[k], segment) / max(np.dot(segment, segment), 1e-9), 0.0, 1.0)
            closest = points[k] + t * segment
            distance = np.hypot(*(position - closest))
            if distance < best[0]:
                best = (distance, k, closest)
        _, k, closest = best
        self.current_waypoint_idx = k
        segment = points[k+1] - points[k]
        tangent = segment / max(np.hypot(*segment), 1e-9)
        offset = position - closest
        cross_track = float(tangent[0] * offset[1] - tangent[1] * offset[0])

        remaining = np.hypot(*(points[k+1] - closest)) + sum(np.hypot(*(points[n+1] - points[n])) for n in range(k+1, len(points) - 1))

        carrot, to_go, n = closest, self.LOOKAHEAD, k
        while n < len(points) - 1:
            step = np.hypot(*(points[n+1] - carrot))
            if step >= to_go:
                carrot = carrot + (points[n+1] - carrot) * (to_go / step)
                break
            to_go -= step
            carrot, n = points[n+1], n + 1
        return carrot, float(remaining), cross_track, tangent

    def turn_rate(self, heading_error_deg:float) -> float:
        '''Turn rate for a heading error: nothing inside the deadband, otherwise the heading PID's output but never
        weaker than MIN_TURN, since a weaker command does not move the simulated robot at all'''
        if abs(heading_error_deg) < self.HEADING_DEADBAND_DEG:
            self.heading_pid.reset()
            return 0.0
        turn = self.heading_pid.step(np.deg2rad(heading_error_deg))
        return float(np.sign(heading_error_deg) * np.clip(abs(turn), self.MIN_TURN, self.MAX_TURN))

    def send_speed(self, target_speed:float, turn:float):
        '''Ramp the forward speed towards target_speed by at most MAX_ACCEL per second, then send it with the turn'''
        limit = self.MAX_ACCEL * self.DT
        self.cmd_speed = float(np.clip(target_speed, self.cmd_speed - limit, self.cmd_speed + limit))
        self.move_2D(self.cmd_speed, 0.0, turn)

    def drive(self, position:np.array):
        '''One control tick along the path, car-style: turn the nose towards the carrot and drive forward, at a
        speed set by how much of the leg is left and cut back the further off-aim the robot is'''
        carrot, remaining, cross_track, tangent = self.follow_path(position)
        left = np.array((-tangent[1], tangent[0])) # Unit vector pointing left of the path
        carrot = carrot + self.cross_track_pid.step(-cross_track) * left
        to_carrot = carrot - position
        if np.hypot(*to_carrot) < 0.25 * self.resolution:
            heading_error = 0.0 # Right on top of the carrot (only happens at the goal): nothing sensible to aim at
        else:
            heading_error = wrap_deg(np.rad2deg(np.arctan2(to_carrot[1], to_carrot[0])) - self.pose[2])

        # 1 when lined up, falling to 0 at TURN_ONLY_DEG, where it just turns on the spot
        aim = float(np.clip((self.TURN_ONLY_DEG - abs(heading_error)) / (self.TURN_ONLY_DEG - self.FULL_SPEED_DEG), 0.0, 1.0))
        speed = float(np.clip(self.distance_pid.step(remaining), self.MIN_SPEED, self.MAX_SPEED))
        target_speed = max(speed * aim, self.MIN_SPEED) if aim > 0.0 else 0.0 # Moving at all means above the deadband
        self.send_speed(target_speed, self.turn_rate(heading_error))

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            return # Does not run if no pose received from Odom or Optitrack
        self.get_logger().debug(f"Pose: {self.pose}")

        if self.path != self._last_printed_path: # Prints once immediately (map + start + goals), then again each time self.path changes
            self.print_map()
            self._last_printed_path = list(self.path)

        ###### INSERT CODE HERE ######
        if self.state == "PLAN":
            # No stop command here: the robot keeps rolling on its last velocity for this one tick, so moving on
            # from a goal is one smooth motion rather than a halt and restart
            self.state = "DRIVE" if self.plan_leg() else "STOP"

        elif self.state == "DRIVE":
            position = np.array(self.body_centre())
            if np.hypot(*(position - self.waypoints[-1])) < self.WAYPOINT_TOL: # Arrived at the goal
                self.goal_reached = True
                self.get_logger().info(f"Reached goal {self.goal_idx+1} at {self.goal_list[self.goal_idx]}, pose {self.pose}")
                self.goal_idx += 1
                self.state = "PLAN" if self.goal_idx < len(self.goal_list) else "STOP"
            else:
                self.drive(position)

        elif self.state == "STOP":
            self.send_speed(0.0, 0.0) # Ramp down to a standstill rather than stopping dead
            if self.cmd_speed == 0.0:
                self.move_2D(0.0, 0.0, 0.0) # Repeated for a few ticks so the stop definitely reaches the robot
                self.stop_ticks += 1
                if self.stop_ticks > 5:
                    self.get_logger().info("All goals reached, shutting down")
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
