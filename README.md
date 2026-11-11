# CSC477 Estimation Assignment (ROS 2 Humble)

Two parts:

1. **Occupancy grid mapping** (ROS 2): build a map from 2D laser scans and
   known (but noisy) poses, recorded from a simulated Husky driving through
   the walls world of Assignment 1.
2. **Least-squares localization** (plain Python): estimate a robot's
   trajectory from range measurements to known landmarks.

The assignment handout (PDF) is the authoritative description of what to
hand in.

## Build

```bash
cd ~/csc477_ws
colcon build --symlink-install
source install/setup.bash
```

With `--symlink-install`, edits to the Python files take effect the next
time you start them; no rebuild is needed.

## Part 1: occupancy grid mapping

Fill in the TODOs in `scripts/occupancy_grid_mapper.py`. Then, in two
terminals:

```bash
# terminal 1: your mapper, plus rviz2 with the course configuration
ros2 launch estimation_assignment occupancy_grid_mapper.launch.py \
    odometry_orientation_noise_std_dev:=2.5 odometry_position_noise_std_dev:=0.1

# terminal 2, in this package's directory: play the recorded data
ros2 bag play laser_and_odometry
```

Start the mapper first, so it sees the recording from the beginning, and
restart it for every new noise setting: the map accumulates for as long as
the node runs. Both noise arguments are required. The orientation noise is
given in **degrees**, and the provided code applies yaw noise with twice
that standard deviation (2.5 gives 5°); the position noise is a standard
deviation in **metres**. `0` is fine for either. Add `rviz:=false` to start
the mapper alone.

The recording (`ros2 bag info laser_and_odometry`) is 138 s long:

| Topic | Type | |
|---|---|---|
| `/husky_1/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz, 270°, 0.1 to 8 m, frame `husky_1/base_laser` |
| `/husky_1/odometry/ground_truth` | `nav_msgs/msg/Odometry` | 100 Hz, pose of `husky_1/base_link` in `map` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | the robot's frames |
| `/clock` | `rosgraph_msgs/msg/Clock` | simulation time of the recording |

The laser is mounted upside down: `husky_1/base_laser` is rotated 180° about
x relative to `husky_1/base_link`, so its z axis points down. The starter
code accounts for this in `laser_scan_callback`; read the comments there.

The mapper publishes, and rviz2 shows:

| Topic | Type | In rviz2 |
|---|---|---|
| `/husky_1/occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | the map: white free, black occupied, grey unknown |
| `/husky_1/debug/laser_points` | `visualization_msgs/msg/Marker` | green: the scan, transformed into `map` by *your* code |
| `/husky_1/debug/robot_pose` | `geometry_msgs/msg/PoseStamped` | red arrow: the laser pose *your* code computed |

rviz2 also draws the raw scan in red, placed by the recorded TF tree. With
zero noise, your green points should land on the red ones. They can trail
a little while the robot turns, because the mapper skips scans that arrive
while it is busy. If they are shifted, rotated or mirrored, your transforms
are wrong. Use the TF display (or
`ros2 run tf2_ros tf2_echo husky_1/base_link husky_1/base_laser`
while the bag plays) to see where the frames are.

Updating the map is slow in Python, so the mapper integrates only some of
the 10 Hz scans (a few per second); that is expected.

### Saving the map

While the mapper is still running (e.g. after the bag has finished):

```bash
ros2 run estimation_assignment save_map.py map_0_firstname_lastname.png
```

### Things that differ from ROS 1

If you have seen the ROS 1 version of this assignment, or are reading older
material:

- `tf.transformations` does not exist in ROS 2. `scripts/transformations.py`
  provides the functions you need with the same names and the same
  `[x, y, z, w]` quaternion order, imported as `tr`.
- The bag is a directory, `laser_and_odometry/`, played with
  `ros2 bag play laser_and_odometry`.
- rviz2 is started by the launch file, with `use_sim_time` set so that it
  follows the recording's clock. If you start it yourself, use
  `rviz2 -d $(ros2 pkg prefix estimation_assignment)/share/estimation_assignment/config/csc477.rviz --ros-args -p use_sim_time:=true`.

## Part 2: least-squares localization

Fill in the TODOs in `scripts/localization.py`, then run it from this
package's directory:

```bash
python3 scripts/localization.py
```

It prints the error of each estimated state and plots the estimated and
true trajectories with the landmarks. The measurements are random, so every
run is different.

## Tests

```bash
cd ~/csc477_ws
colcon test --packages-select estimation_assignment && colcon test-result --verbose
```

These check the provided code and the recording, not your TODOs.
