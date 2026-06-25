# pct_art_local_navigation

ROS 2 integration package for the following navigation chain, without Nav2:

```text
PCT /pct_path -> ART local planning -> /local_path -> Pure Pursuit -> /cmd_vel -> Go2
```

Prerequisites supplied by the localization stack:

- `/Odometry_open3d`
- `/scan_base_link`
- TF `map -> base_link`

Build and launch:

```bash
colcon build --symlink-install --packages-up-to pct_art_local_navigation
source install/setup.bash
ros2 launch pct_art_local_navigation pct_art_local_navigation.launch.py \
  network_interface:=enp2s0
```

The Go2 bridge starts disabled. Inspect localization, map, ART path and status,
then enable motion explicitly:

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'
```

If the PCT planner is already running, add `start_pct_planner:=false`.

The launch file adds the standard Unitree SDK install directory
`/opt/unitree_robotics/lib` to `LD_LIBRARY_PATH` for the Go2 bridge.

Goal completion and heading control parameters are kept in this package's YAML
files:

- `config/coordinator.yaml`: `goal_reached_distance` (metres) and
  `goal_yaw_tolerance` (radians), plus ART path post-processing parameters
  `enable_path_smoothing`, `path_min_point_spacing`, `path_resample_spacing`,
  `path_collinear_angle_threshold`, `path_smoothing_iterations`,
  `path_smoothing_data_weight`, `path_smoothing_smooth_weight`, and
  `path_max_deviation`.
- `config/pure_pursuit_local.yaml`: `rotate_to_path_threshold`,
  `rotate_to_path_tolerance`, `goal_yaw_tolerance`, and
  `rotate_to_heading_gain`.

The coordinator latches `GOAL_REACHED` and forgets the completed PCT task after
both position and yaw are satisfied. Pure Pursuit then keeps publishing zero
velocity for the empty path, so the bridge can remain enabled and a later new
PCT path starts a new task without reviving the old goal.

ART publishes its raw feasible polyline on `/art_planner/path`. The coordinator
optionally cleans, smooths, resamples and revalidates it before publishing
`/local_path` for Pure Pursuit. If the smoothed path leaves the local GridMap
margin or deviates too far from the ART path, the coordinator publishes an empty
path and enters `BLOCKED` instead of falling back to the raw ART path.
