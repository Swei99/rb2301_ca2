# RB2301 CA2 – Robot Path Planning (Simulation)

In this assignment your robot must drive through a known maze, visiting a list of goal points **in order**, without touching any walls. You will:

1. **Plan** a collision-free path on the occupancy grid (e.g. with A\*, Dijkstra or BFS).
2. **Track** that path with the robot using waypoints and PID control.

You develop everything in the Gazebo simulation first. The real-robot part is described at the end of this file.

---

## Where your solution goes

**Everything you write goes in one file:**

```
src/rb2301_ca2/rb2301_ca2/path_planning.py
```

- Put your **path-planning + PID logic** in `WaypointNode.timer_callback()`, between the two `###### INSERT CODE HERE ######` markers. It runs 20 times per second.
- Put any **variables you need to remember between calls** (current goal index, waypoint list, PID error terms, ...) in `WaypointNode.__init__()`.
- You may add your own helper functions/methods to the file.

Do not edit the other files (launch files, world, map, scripts).

### What you are given (inside `WaypointNode`)

| Name | What it is |
|---|---|
| `self.pose` | Latest robot pose as `[x, y, heading]` (metres, metres, degrees from -180 to 180; 0 degrees points along the +x axis and turning anticlockwise increases the heading). `None` until the first reading arrives. |
| `self.goal_list` | The goals to reach, in order, as `(x, y)` world coordinates. |
| `self.map_array` | The occupancy grid (35 x 30 numpy array). `0` = free, `99` = wall. |
| `self.origin`, `self.resolution` | Where the grid sits in the world and the size of one cell (see below). |
| `self.move_2D(x, y, turn)` | Command the robot: forward/backward speed `x`, sideways speed `y`, rotation speed `turn`. Values are clipped to the robot's maximum speed. |
| `self.path` | Set this to your planned route (a list of grid cells) and it is drawn in the terminal automatically (see below). |
| `grid_to_world(i, j, origin, resolution)` | Grid cell `(i, j)` -> world `(x, y)` of the **centre** of that cell. |
| `world_to_grid(x, y, origin, resolution)` | World `(x, y)` -> grid cell `(i, j)` that contains that point. |
| `Grid` class | Helpers: `check_grid_validity()` (flood-fill: is a route possible?), `draw_grid_map()`, `print_grid_map()`, `animate_path()`. |

Use `self.origin` and `self.resolution` in the two conversion functions, e.g. `world_to_grid(x, y, self.origin, self.resolution)`.

### How the grid maps to the world

- The grid is a numpy array indexed `[i, j]`; `i` increases with world **x**, `j` increases with world **y**.
- Each cell is 0.2 m x 0.2 m. The grid's corner (the corner of cell `[0, 0]`) is at world `(-1.0, -5.0)`.
- A cell's world position is its **centre**: cell `[0, 0]` is centred at `(-0.9, -4.9)`, cell `[1, 0]` at `(-0.7, -4.9)`, and so on. The provided conversion functions already handle this, so use them rather than re-deriving it.
- Walls in the map already include a safety buffer, so travelling through free cells is safe.

### Goals in the simulation

The robot starts at world `(0.0, 0.0)`. The goals, in order, are:

```
(3.5, -3.5)  ->  (3.3, 0.3)  ->  (2.5, -3.5)  ->  (-0.3, -3.7)
```

### See your map and route in the terminal

When you run the node, the map is printed automatically as soon as the robot's pose is received:

```
#  wall        .  free cell        S  the robot's current cell
W  goal point  G  final goal       *  your planned route
```

As soon as you assign `self.path = [(i1, j1), (i2, j2), ...]` (list of grid cells, in travel order), the map is reprinted with your route drawn on it. This is a quick way to sanity check your planner before you drive the robot. For a picture you can also call `Grid(...).draw_grid_map(path=..., waypoints=..., save_path="route.png")`.

### Movement notes

- The robot is omnidirectional, so `move_2D` accepts a sideways speed `y` as well.
- **Bonus:** reach the goals using only forward/backward motion and turning (no sideways `y` speed), like a normal wheeled car.
- Simulation speed limit is 1.4 m/s (enforced inside `move_2D`).

---

## Setup

### Requirements

- Ubuntu with **ROS 2 Jazzy** and **Gazebo Harmonic**
- The ROS-Gazebo bridge packages (`ros-jazzy-ros-gz`)
- Python packages: `numpy`, `pillow`
- Your existing **RB2301 workspace** (the folder that contains `src/`, e.g. `~/Documents/rb2301`)

### Add the CA2 files to your RB2301 workspace

1. Download this repository (green **Code** button -> *Download ZIP* and extract it, or `git clone https://github.com/rudra-8000/rb2301_ca2.git`).
2. Copy the two folders inside its `src/` into the `src/` folder of **your RB2301 workspace**:
   - `rb2301_ca2/`
   - `rb2301_gz/` (if you already have an `rb2301_gz` from CA1, delete it first and replace it with this one, it contains the CA2 world as well)
3. Copy the two scripts `ca2.sh` and `gz_ca2.sh` into the **workspace folder itself** (next to `src/`, not inside it), then run `chmod +x ca2.sh gz_ca2.sh` there.

Your workspace should now look like this:

```
rb2301/                      <- your workspace folder
  ca2.sh
  gz_ca2.sh
  src/
    rb2301_ca2/              <- edit rb2301_ca2/rb2301_ca2/path_planning.py
    rb2301_gz/
    ... (your other packages)
```

Example, assuming you cloned into `~/rb2301_ca2_download` and your workspace is `~/Documents/rb2301`:

```bash
cd ~/Documents/rb2301
rm -rf src/rb2301_gz src/rb2301_ca2          # only if these already exist
cp -r ~/rb2301_ca2_download/src/rb2301_ca2 ~/rb2301_ca2_download/src/rb2301_gz src/
cp ~/rb2301_ca2_download/ca2.sh ~/rb2301_ca2_download/gz_ca2.sh .
chmod +x ca2.sh gz_ca2.sh
```

### Build

Run from your workspace folder:

```bash
cd ~/Documents/rb2301
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Because of `--symlink-install`, edits to `path_planning.py` take effect without rebuilding. You only need to rebuild if you add/remove files.

### Run

Always run these from your workspace folder.

**Terminal 1 - start the simulation:**

```bash
./gz_ca2.sh
```

Wait until the Gazebo window opens and the maze and robot appear. The robot spawns at `(0, 0)`. (To spawn somewhere else, edit the `x:=` and `y:=` values in `gz_ca2.sh`.)

**Terminal 2 - run your code:**

```bash
./ca2.sh
```

Stop either with `Ctrl+C`. Stopping the simulation with `Ctrl+C` also cleans up leftover Gazebo processes automatically.

---

## Troubleshooting

- **`Permission denied` running a `.sh` file:** run `chmod +x gz_ca2.sh ca2.sh`.
- **Gazebo stuck / keeps printing "Requesting list of world names":** a previous Gazebo is probably still running in the background. Run `pkill -9 ruby`, then start `./gz_ca2.sh` again.
- **The robot does not move / nothing is printed by `./ca2.sh`:** the simulation must be fully loaded first. Check that Terminal 1 is still running, then rerun `./ca2.sh`.
- **`ros2: command not found` / package not found:** source ROS (`source /opt/ros/jazzy/setup.bash`) and make sure `colcon build` finished without errors. The scripts source `install/setup.bash` for you, so they must be run from the workspace folder (the one containing `install/`).
- **Gazebo window is black, very slow, or crashes (virtual machine / no GPU):** open `src/rb2301_gz/launch/ca2_gazebo.launch.py` and change `SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "0")` to `"1"`, then restart the simulation (no rebuild needed).

---

## Real robot (branch `irl-final-maze`, being tested)

Same `path_planning.py`, same code: the only differences are the pose source (Optitrack instead of Gazebo odometry), the map and the goals. Your code does not change between simulation and the real maze, because the pose is converted into the maze's own frame **before** your code sees it (`self.origin` is `(0, 0)`, `self.resolution` is `0.2` as in simulation, `world_to_grid()` / `grid_to_world()` work the same).

### The real maze

A fixed 3.2 m x 3.2 m maze, built at the **same 0.2 m grid** as the simulation (`ca2_irl_map.npy`, 16 x 16, `0` = free, `99` = wall). Coordinates are in metres, with `(0, 0)` at the outer corner of array cell `[0, 0]`, `+x` along array axis 0 and `+y` along axis 1. There are three runs, picked with `--run`:

| `--run` | Run | Robot starts at (x, y) | Goals, in order |
|---|---|---|---|
| `test1` | Test run 1 | (0.5, 0.5) | (2.9, 1.1) -> (1.7, 0.5) -> (2.3, 0.5) -> (1.7, 1.1) |
| `test2` | Test run 2 | (2.7, 2.7) | (0.5, 1.1) -> (1.7, 2.3) -> (0.5, 2.9) -> (1.1, 2.9) |
| `full` | Full run | (0.5, 2.1) | (1.1, 0.5) -> (2.3, 2.9) -> (2.9, 0.5) -> (0.5, 1.7) |

Place the robot on its start point before launching. (The first pose is checked against the run's start and you get a warning if it is more than 0.4 m away.) The robot's speed is capped at 0.3 m/s.

### 1. One-time setup per maze placement: `optitrack_variables.config`

This is the **only** thing to edit when the maze is moved. It tells the code where the maze sits in Optitrack's frame (`src/rb2301_ca2/rb2301_ca2/optitrack_variables.config`):

```ini
[frame]
origin_x = 0.0        ; Optitrack (x, y) in metres of the maze's (0, 0) corner
origin_y = 0.0
rotation_deg = 0.0    ; angle of the maze's +x axis in Optitrack's frame, counter-clockwise positive

[robot]
number = 3            ; your Bingda robot: 3 -> /vrpn_mocap/bingda_003/pose  (or pass --robot N)
```

### 2. Calibrate

Don't work these out by hand:

1. On the robot, start Optitrack (below), then run `./ca2.sh --run test1 --calibrate`. It prints the raw Optitrack pose and the maze-frame pose twice a second and **never sends a velocity command**, so you can push the robot around by hand.
2. Put the robot (or any rigid body Optitrack tracks) on two or more known maze points, far apart (for example the start point of `test1` and a point near the opposite corner of the maze), and note the Optitrack `(x, y)` shown at each.
3. On any computer (no ROS needed):
   ```bash
   python3 tools/irl_calibrate.py --pair 0.5 0.5 <opti_x> <opti_y> --pair 2.9 2.9 <opti_x> <opti_y>
   ```
   Each `--pair` is `maze_x maze_y optitrack_x optitrack_y`. It prints the three numbers to paste into the config and a fit residual (it should be a centimetre or two; it warns above 5 cm).
4. Copy the config to the robot (step 3 below), run `--calibrate` again and check the **maze** pose matches where you really put the robot. Also turn the robot to face the maze's `+x` direction: the maze heading should read about `0`.

Motive must stream with **Z-up** so that `x, y` are the floor plane (it is, for the existing set-up).

### 3. Run it on the robot

The robot runs Ubuntu 20.04 with ROS 2 **Foxy** (Python 3.8). Your laptop only has to copy files to it:

```bash
# on your laptop, from this repository (use your own folder name; --delete only touches that one package folder)
ssh bingda@192.168.1.20x "mkdir -p ~/Downloads/rb2301_ca2_<YourName>/src"
rsync -auvx --delete src/rb2301_ca2 bingda@192.168.1.20x:~/Downloads/rb2301_ca2_<YourName>/src

# on the robot (ssh in), once and after every rsync that adds/removes files
cd ~/Downloads/rb2301_ca2_<YourName> && colcon build --symlink-install
```

(`x` is your robot's number. Re-run the `rsync` after editing `path_planning.py` or the config on your laptop.) Then, in **separate terminals on the robot**:

```bash
ros2 launch base_control_ros2 base_control.launch.py     # alias: basecontrol   -- reads /cmd_vel, drives the wheels
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.199 port:=3883   # alias: vrpn   -- publishes Optitrack poses
cd ~/Downloads/rb2301_ca2_<YourName> && ./ca2.sh --run test1       # or test2 / full; add --robot N if it isn't robot 3
```

The `base_control_ros2` / `vrpn` / WiFi details are in the lab handout. Check that `ros2 topic list` shows `/vrpn_mocap/bingda_00x/pose` before starting. Keep a hand near the robot and `Ctrl+C` the `ca2.sh` terminal if it misbehaves.

### Troubleshooting (real robot)

- **No pose / nothing prints:** the node only starts working once it receives a pose. Is `vrpn` running, and does the rigid body in Motive have the same name as the topic (`bingda_00x`)? Is the robot's `ROS_DOMAIN_ID` set to its number?
- **`Robot is at ... m away` warning:** the robot is not on the start point, or the three numbers in the config are wrong. Re-run `--calibrate`.
- **Printed map shows the robot in the wrong cell, or the robot "drives into walls":** check the rotation first (a sign or 90 degree error is the usual cause), then the origin.
- **Heading is off by a fixed angle:** the rigid body was created facing a different direction from the robot's front. Recreate it in Motive with the robot facing the Optitrack `+x` axis, or ask the TAs.
- `--maze 0/1` from an older version of this README no longer exists; use `--run test1|test2|full`.

---

## What is in this repository

```
ca2.sh, gz_ca2.sh              copy next to src/ in your workspace
src/rb2301_ca2/                copy into your workspace's src/
  rb2301_ca2/path_planning.py  <-- YOUR SOLUTION GOES HERE
  rb2301_ca2/ca2_sim_map.npy   occupancy grid of the simulation maze
  rb2301_ca2/ca2_irl_map.npy, ca2_irl_layout.json   real maze: grid, and start/goal points of the 3 runs (fixed)
  rb2301_ca2/optitrack_variables.config   real maze placement in the Optitrack frame (the only file to edit)
src/rb2301_gz/                 copy into your workspace's src/ (Gazebo world, robot model, launch file; do not edit)
tools/irl_calibrate.py         real robot only: solves origin/rotation for the config from a few Optitrack readings (no ROS)
```
