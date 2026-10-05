# RB2301 CA2 – Robot Path Planning

Your robot must drive through a known maze and visit a list of goal points **in order**, without touching a wall. You will:

1. **Plan** a collision-free path on the occupancy grid (e.g. A\*, Dijkstra or BFS).
2. **Track** that path with waypoints and PID control.

You develop and test in the **Gazebo simulation** first (sections 1-3), then run the same code on the **real robot** in the lab maze (section 4). The code does not change between the two.

---

## 1. Where your solution goes

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
| `self.pose` | Latest robot pose as `[x, y, heading]` (metres, metres, degrees from -180 to 180; 0 degrees points along the +x axis and turning anticlockwise increases the heading). `None` until the first reading arrives. Same meaning in the simulation and on the real robot. |
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

### Goals in the simulation (real-maze goals are in section 4)

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
- Speed limits (enforced inside `move_2D`): 1.4 m/s in the simulation, **0.3 m/s on the real robot**. Do not change them.

---

## 2. Set up your laptop

**Requirements:** Ubuntu with **ROS 2 Jazzy** and **Gazebo Harmonic**, the ROS-Gazebo bridge (`ros-jazzy-ros-gz`), Python packages `numpy` and `pillow`, and your existing **RB2301 workspace** (the folder that contains `src/`, e.g. `~/Documents/rb2301`).

1. Get this repository (`git clone -b irl-final-maze https://github.com/rudra-8000/rb2301_ca2.git`, or *Code -> Download ZIP*).
2. Copy the two folders in its `src/` into **your workspace's** `src/`: `rb2301_ca2/` and `rb2301_gz/` (if you already have an `rb2301_gz` from CA1, replace it: it contains the CA2 world too).
3. Copy `ca2.sh` and `gz_ca2.sh` into the **workspace folder itself** (next to `src/`) and `chmod +x ca2.sh gz_ca2.sh`.

```bash
cd ~/Documents/rb2301                      # your workspace
rm -rf src/rb2301_gz src/rb2301_ca2        # only if they already exist
cp -r ~/rb2301_ca2/src/rb2301_ca2 ~/rb2301_ca2/src/rb2301_gz src/
cp ~/rb2301_ca2/ca2.sh ~/rb2301_ca2/gz_ca2.sh . && chmod +x ca2.sh gz_ca2.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Edits to `path_planning.py` take effect without rebuilding (`--symlink-install`); rebuild only if you add or remove files.

---

## 3. Run in the simulation

From your workspace folder, in two terminals:

```bash
./gz_ca2.sh      # terminal 1: Gazebo; wait until the maze and robot appear (robot spawns at (0, 0))
./ca2.sh         # terminal 2: your path_planning node
```

Stop with `Ctrl+C` (stopping Gazebo also cleans up leftover Gazebo processes). To spawn the robot elsewhere, edit `x:=` / `y:=` in `gz_ca2.sh`.

---

## 4. Run on the real robot

### The maze and the runs

A fixed 3.2 m x 3.2 m maze on the **same 0.2 m grid** as the simulation (`ca2_irl_map.npy`, 16 x 16, `0` free / `99` wall). Coordinates are in metres, `(0, 0)` at the outer corner of cell `[0, 0]`, `+x` along array axis 0 and `+y` along axis 1. Pick a run with `--run`:

| `--run` | Robot starts at | Goals, in order |
|---|---|---|
| `test1` | (0.5, 0.5) | (2.9, 1.1) -> (1.7, 0.5) -> (2.3, 0.5) -> (1.7, 1.1) |
| `test2` | (2.7, 2.7) | (0.5, 1.1) -> (1.7, 2.3) -> (0.5, 2.9) -> (1.1, 2.9) |
| `full`  | (0.5, 2.1) | (1.1, 0.5) -> (2.3, 2.9) -> (2.9, 0.5) -> (0.5, 1.7) |

Put the robot on its run's start point before launching (a warning is printed if it is more than 0.4 m away). Your code needs no changes: the Optitrack pose is converted into the maze's own frame before you see it, so `self.origin` is `(0, 0)`, `self.resolution` is `0.2`, and `world_to_grid()` / `grid_to_world()` work as in the simulation.

**The TAs set up the Optitrack mapping once and ship it in the package (`optitrack_variables.config`). Do not edit it.** Your robot's number is taken from the robot's `ROS_DOMAIN_ID`, so you never type it. One rule for you: **a robot's rigid body must be created in Motive with the robot facing +y** (up the left side of the printed map, from (0.5, 0.5) towards (0.5, 2.1)); the code then reports heading `+90` when the robot faces `+y` and `0` when it faces `+x`. If you ever re-create or reset a rigid body, face the robot +y first.

### Step by step

The robot runs Ubuntu 20.04 and ROS 2 **Foxy** (Python 3.8); you edit on your laptop and copy the package over. Only `src/rb2301_ca2` goes to the robot, so you start the node with `ros2 run`, **not** `ca2.sh`. Robot number `NN` is two digits and its address is `192.168.1.2NN` (robot 7 -> `192.168.1.207`, robot 12 -> `192.168.1.212`). Use your own name in the folder so packages don't clash.

**1. Create your workspace on the robot** (ssh in as usual):

```bash
mkdir -p ~/Downloads/rb2301_ca2_<YourName>/src
```

**2. Copy your package** from your **laptop's own terminal** (not the ssh one). Repeat after every edit of `path_planning.py`:

```bash
rsync -auvx --delete ~/Documents/rb2301/src/rb2301_ca2 bingda@192.168.1.2NN:~/Downloads/rb2301_ca2_<YourName>/src
```

(Use the path to your own `src/rb2301_ca2`. `--delete` only touches that one folder on the robot.)

**3. Build on the robot** (once, and again only if you add or remove files):

```bash
cd ~/Downloads/rb2301_ca2_<YourName>
colcon build --symlink-install        # alias: colb
```

**4. Run, each in its own ssh terminal on the robot:**

```bash
basecontrol                           # terminal 1: reads /cmd_vel and drives the wheels
vrpn                                  # terminal 2: Optitrack pose client; check: ros2 topic list shows /vrpn_mocap/bingda_0NN/pose
cd ~/Downloads/rb2301_ca2_<YourName> && source install/setup.bash && ros2 run rb2301_ca2 path_planning --run test1     # terminal 3
```

Use `--run test2` or `--run full` for the other runs. The node prints the robot it found (`robot=bingda_0NN`), the map with the robot `S`, goals `W`/`G` and your route `*`, and a warning if the robot is not on the start point.

> **Safety.** The unchanged starter code drives straight ahead at 0.3 m/s as soon as it gets a pose. Start the node only when the way ahead is clear, keep a hand on the robot, and press `Ctrl+C` in the `path_planning` terminal (or in the `basecontrol` terminal) to stop it.

**Optional check before driving:** add `--calibrate`. It prints the raw Optitrack pose and your maze-frame pose twice a second and **never sends a velocity command**, so you can carry the robot around by hand. On the start point the maze pose should match it (within a few cm); facing `+x` the heading reads about `0`. If not, tell a TA.

### Aliases and shortcuts already on the robot

The robots' `~/.bashrc` already defines:

| Alias | Runs |
|---|---|
| `basecontrol` | `ros2 launch base_control_ros2 base_control.launch.py` (wheel driver) |
| `vrpn` | `ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.199 port:=3883` (Optitrack poses) |
| `colb` | `colcon build --symlink-install` |
| `keyboard` | `ros2 run teleop_keyboard keyboard` (drive by hand, to move the robot onto its start point) |
| `rplidar` | `ros2 launch rplidar_ros rplidar.launch.py` (lidar, not needed here) |
| `talker`, `listener` | ROS 2 demo nodes (to test the network) |
| `nanobash`, `sourcebash` | `sudo nano ~/.bashrc` to edit it, `source ~/.bashrc` to reload it |
| `unsetpath` | clears `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` (use only if a build picks up the wrong workspace) |

**Adding your own:** run `nanobash`, add a line at the **bottom** such as

```bash
alias ca2='cd ~/Downloads/rb2301_ca2_<YourName> && source install/setup.bash && ros2 run rb2301_ca2 path_planning --run test1'
```

save, then `sourcebash` (or open a new terminal). Do **not** change the `source ...` lines or `export ROS_DOMAIN_ID=...` in that file (the domain ID is the robot's number and is how your robot is told apart from the others). The robot's `.bashrc` is shared by everyone using that robot, so give your alias a distinctive name and keep other people's lines untouched. A handy one for your **laptop's** `~/.bashrc`:

```bash
alias ca2sync='rsync -auvx --delete ~/Documents/rb2301/src/rb2301_ca2 bingda@192.168.1.2NN:~/Downloads/rb2301_ca2_<YourName>/src'
```

### Troubleshooting (real robot)

- **Nothing prints / the robot never moves:** the node waits for a pose. Is `vrpn` running and does `ros2 topic list` show `/vrpn_mocap/bingda_0NN/pose`? Is `basecontrol` running? Does the start-up line show your robot (`robot=bingda_0NN`)? If not, add `--robot NN`.
- **`Robot is at ... m away`:** the robot is not on the start point. If it is, ask a TA.
- **Robot appears in the wrong cell / drives into walls / turns the wrong way:** ask a TA (the mapping or the rigid body needs checking). Do not edit the config yourself.
- **Build errors or old behaviour:** rsync again, then `colb` on the robot.
- **`--maze` error:** that flag was replaced; use `--run test1|test2|full`.

---

## Troubleshooting (simulation)

- **`Permission denied` running a `.sh` file:** `chmod +x gz_ca2.sh ca2.sh`.
- **Gazebo stuck / keeps printing "Requesting list of world names":** a previous Gazebo is still running. Run `pkill -9 ruby`, then `./gz_ca2.sh` again.
- **The robot does not move / nothing is printed by `./ca2.sh`:** the simulation must be fully loaded first; check Terminal 1 is still running, then rerun `./ca2.sh`.
- **`ros2: command not found` / package not found:** `source /opt/ros/jazzy/setup.bash` and make sure `colcon build` finished without errors. The scripts source `install/setup.bash` themselves, so run them from the workspace folder (the one containing `install/`).
- **Gazebo window is black, very slow or crashes (virtual machine / no GPU):** in `src/rb2301_gz/launch/ca2_gazebo.launch.py` change `SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "0")` to `"1"` and restart the simulation.

---

## What is in this repository

```
ca2.sh, gz_ca2.sh              copy next to src/ in your workspace (simulation)
src/rb2301_ca2/                copy into your workspace's src/ (and rsync to the robot)
  rb2301_ca2/path_planning.py  <-- YOUR SOLUTION GOES HERE
  rb2301_ca2/ca2_sim_map.npy   simulation maze grid
  rb2301_ca2/ca2_irl_map.npy, ca2_irl_layout.json   real maze: grid, and start/goals of the three runs (fixed)
  rb2301_ca2/optitrack_variables.config             maze placement + heading offsets, set by the TAs (do not edit)
src/rb2301_gz/                 copy into your workspace's src/ (Gazebo world, robot model; do not edit)
tools/irl_calibrate.py         TA use: solves the maze origin/rotation from a few Optitrack readings (no ROS)
```

### For TAs / instructors

Calibrate once per maze placement (and after any move of the maze or Optitrack): put a robot on two or more known maze points far apart, read the raw pose with `--calibrate`, then `python3 tools/irl_calibrate.py --pair 0.5 0.5 <opti_x> <opti_y> --pair 2.7 2.7 <opti_x> <opti_y>` (the residual should be 1-2 cm; it warns above 5 cm) and paste `origin_x`, `origin_y`, `rotation_deg` into `[frame]` of `optitrack_variables.config`. Motive must stream **Z-up**. Rigid bodies are created facing maze `+y`, so the heading offset is `auto`; a robot whose rigid body was created differently gets an entry under `[heading_offsets]` (`<robot number> = <reported heading minus true maze heading>`; robots 7 and 12 are measured).
