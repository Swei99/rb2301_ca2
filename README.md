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

### 1. The maze mapping (set by the TAs: do not edit)

The TAs calibrate Optitrack once and ship the result inside the package, in `src/rb2301_ca2/rb2301_ca2/optitrack_variables.config`. It tells the code where the maze sits in Optitrack's frame (three numbers: `origin_x`, `origin_y`, `rotation_deg`), so that every pose is converted to the maze frame before your code sees it. **Use the config you are given as it is.** If positions or headings look wrong, tell a TA instead of changing the numbers.

Your robot's number is picked up automatically from `ROS_DOMAIN_ID` (the lab robots set it to their number). Only add `--robot x` if the node reports the wrong robot.

**The robot must face +y when its rigid body is created.** In Motive, a rigid body is created (or its orientation reset) with the robot sitting still, and the robot's front must point in the maze's **+y** direction at that moment (from the start of `test1`, (0.5, 0.5), towards the start of the `full` run, (0.5, 2.1): "up" the left side of the map printed in your terminal). The package assumes this, so `self.pose[2]` is `+90` when the robot faces maze `+y` and `0` when it faces `+x`. If you ever re-create or reset your robot's rigid body, put the robot facing +y first (a few degrees either way is fine), otherwise its heading will be wrong and the robot will turn the wrong way.

### 2. Check where the robot thinks it is (optional, before driving)

Run your node with `--calibrate`. It prints the raw Optitrack pose and the maze-frame pose twice a second and **never sends a velocity command**, so you can push the robot around by hand:

```bash
source install/setup.bash && ros2 run rb2301_ca2 path_planning --run test1 --calibrate
```

Put the robot on the run's start point: the **maze** pose should match it (within a few cm) and the printed cell should be right. Turn the robot to face the maze's `+x` direction: the maze heading should read about `0`. If not, tell a TA before driving.

### 3. Run it on the robot

The robot runs Ubuntu 20.04 with ROS 2 **Foxy** (Python 3.8). You write and edit on your laptop, then copy the package to the robot. Only `src/rb2301_ca2` goes to the robot, so on the robot you start the node with `ros2 run`, **not** `ca2.sh`.

1. Connect your laptop to the lab network (Wi-Fi name and password are in the lab handout / slides).
2. SSH to the robot and create your own workspace (replace `x` with your robot's number, e.g. `x=2` for BINGDA-002; the robot password is in the slides; use your own name so packages don't clash):
   ```bash
   ssh bingda@192.168.1.20x
   mkdir -p ~/Downloads/rb2301_ca2_<YourName>/src
   ```
3. From your laptop's **own** terminal (not the ssh one), rsync the package to the robot. Re-run this every time you edit `path_planning.py`:
   ```bash
   rsync -auvx --delete ~/Documents/rb2301_ca2/src/rb2301_ca2 bingda@192.168.1.20x:~/Downloads/rb2301_ca2_<YourName>/src
   ```
   (Use the path of your own `src/rb2301_ca2`. `--delete` only affects that one package folder on the robot.)
4. Build on the robot (in the ssh terminal):
   ```bash
   cd ~/Downloads/rb2301_ca2_<YourName>
   colcon build --symlink-install
   ```
5. Run each of these in a **separate** ssh terminal on the robot:
   ```bash
   ros2 launch base_control_ros2 base_control.launch.py      # alias: basecontrol -- reads /cmd_vel and drives the wheels
   vrpn                                                      # alias for: ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.199 port:=3883
   cd ~/Downloads/rb2301_ca2_<YourName> && source install/setup.bash && ros2 run rb2301_ca2 path_planning --run test1
   ```
   Use `--run test2` or `--run full` for the other runs. Put the robot on the run's start point first.

   **The unchanged starter code drives the robot straight ahead at 0.3 m/s as soon as it gets a pose.** Start the node only when the way ahead is clear, keep a hand on the robot, and press `Ctrl+C` in the `path_planning` terminal (or `Ctrl+C` in the `base_control` terminal) to stop it. Once your own code is in `timer_callback()`, the same speed cap of 0.3 m/s applies.

Check that `ros2 topic list` shows `/vrpn_mocap/bingda_00x/pose` before starting. Keep a hand near the robot and press `Ctrl+C` in the `path_planning` terminal if it misbehaves.

### For TAs / instructors: calibrating the maze mapping

Do this once per maze placement (and again if the maze, Optitrack or a rigid body is moved or rebuilt), then give students the package with the config filled in:

1. Start `vrpn`, then run `ros2 run rb2301_ca2 path_planning --run test1 --calibrate` with the robot on two or more known maze points, far apart (for example the `test1` start (0.5, 0.5) and the `test2` start (2.7, 2.7)). Note the raw Optitrack `(x, y)` at each.
2. On any computer (no ROS needed): `python3 tools/irl_calibrate.py --pair 0.5 0.5 <opti_x> <opti_y> --pair 2.7 2.7 <opti_x> <opti_y>`. Each `--pair` is `maze_x maze_y optitrack_x optitrack_y`. The fit residual should be a centimetre or two (it warns above 5 cm).
3. Paste `origin_x`, `origin_y`, `rotation_deg` into `[frame]` in `optitrack_variables.config`. Rerun `--calibrate` and check the maze pose equals where you put the robot.
4. **Heading:** every rigid body is created with the robot facing maze **+y** (the same direction for all robots), and the package then reports `heading = Motive heading + 90` (`heading_offset_deg = auto`). So facing maze `+x` reads `0` and facing `+y` reads `+90`. Check each robot by facing it along `+x` and reading `--calibrate`, or by driving it forward a few cm: the reported heading must match the direction it really moves. A rigid body that was not created facing +y is fixed either by re-creating it in Motive, or by a per-robot entry under `[heading_offsets]` in the config, `<robot number> = <degrees>`, where degrees = reported heading minus the true maze heading (robots 7 and 12 have measured entries).
5. Motive must stream with **Z-up** so that `x, y` are the floor plane.

### Troubleshooting (real robot)

- **No pose / nothing prints:** the node only starts working once it receives a pose. Is `vrpn` running, and does the rigid body in Motive have the same name as the topic (`bingda_00x`)? Is the robot's `ROS_DOMAIN_ID` set to its number? The node prints the robot it uses at start-up (`robot=bingda_00x`); pass `--robot x` if that is wrong.
- **`Robot is at ... m away` warning:** the robot is not on the start point, or the maze mapping in the config is out of date. Check the placement with `--calibrate`, then ask a TA.
- **Printed map shows the robot in the wrong cell, or the robot "drives into walls":** this is a mapping problem, so ask a TA (do not edit the config yourself).
- **Heading is off by a fixed angle:** the rigid body was not created with the robot facing maze +y (see "The robot must face +y"). Ask a TA to re-create it, or to add a `[heading_offsets]` entry.
- `--maze 0/1` from an older version of this README no longer exists; use `--run test1|test2|full`.

---

## What is in this repository

```
ca2.sh, gz_ca2.sh              copy next to src/ in your workspace
src/rb2301_ca2/                copy into your workspace's src/
  rb2301_ca2/path_planning.py  <-- YOUR SOLUTION GOES HERE
  rb2301_ca2/ca2_sim_map.npy   occupancy grid of the simulation maze
  rb2301_ca2/ca2_irl_map.npy, ca2_irl_layout.json   real maze: grid, and start/goal points of the 3 runs (fixed)
  rb2301_ca2/optitrack_variables.config   real maze placement in the Optitrack frame (set by the TAs; do not edit)
src/rb2301_gz/                 copy into your workspace's src/ (Gazebo world, robot model, launch file; do not edit)
tools/irl_calibrate.py         TA use: solves origin/rotation for the config from a few Optitrack readings (no ROS)
```
