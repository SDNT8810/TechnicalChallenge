# Simulation

This explains how `sdnt_robot_simulation` brings up the world, robot, and Nav2 stack used by the waypoint follower and blockchain logger.

## Package Pieces
- World: `world/warehouse.world`
- Map: `maps/warehouse.yaml` + `warehouse_map.pgm`
- URDF: `src/description/sdnt_robot_description.urdf`
- Config: `config/nav2_params.yaml`, `config/ekf.yaml`
- RViz: `rviz/display.rviz`
- Launch: `launch/simulation.launch.py`

## Launch Sequence (simulation.launch.py)
1. Declare args (gui, model, rvizconfig, use_sim_time, map).
2. Start `gzserver` with `warehouse.world` (ROS init + factory plugins).
3. Generate robot description via `xacro` and start `robot_state_publisher`.
4. Start `joint_state_publisher` (GUI off by default).
5. Spawn entity `sdnt_robot` at (0,0,0.18).
6. Start EKF (`robot_localization`) using `ekf.yaml` (2D mode, 30 Hz, odom + imu inputs, world frame `odom`).
7. Publish static transform `map -> odom` (identity) via `static_transform_publisher`.
8. Launch map server (lifecycle) serving the occupancy grid defined in `warehouse.yaml`.
9. Launch AMCL (lifecycle) for localization (`scan_topic: /scan`).
10. Include Nav2 bringup (`navigation_launch.py`) with rewritten `nav2_params.yaml` (autostart True, slam False).
11. Start RViz2 with `rviz/display.rviz`.
12. Lifecycle manager handles `map_server` auto transitions.

## Nav2 Configuration Highlights
- Planner: `NavfnPlanner` (A* enabled, tolerance 0.5 m).
- Controller: DWB (`FollowPath`) with linear ±1.0 m/s, angular up to 1.5 rad/s, critics (GoalDist, PathDist, PathAlign, GoalAlign, RotateToGoal, PreferForward, etc.).
- Goal checking: XY tolerance 0.30 m, yaw tolerance ~2π (ignore orientation) for fast acceptance.
- Local costmap: 3m x 3m rolling window, 0.05 m resolution, voxel + inflation layers.
- Global costmap: static + obstacle + inflation layers, 0.05 m resolution.
- Recovery behaviors: spin, backup, wait.
- AMCL: 500–2000 particles, likelihood field laser model, `base_link` / `odom` / `map` frames aligned with static TF.

## EKF (robot_localization)
- Frequency 30 Hz, `two_d_mode: true`.
- Fuses `/odom` (position + yaw) and `/imu` (angular velocities).
- `publish_tf: false`; static map->odom transform supplied separately.

## Waypoints & Follower Interaction
- Waypoints CSV in `follow_waypoints_pkg` provides 3 map-frame goals.
- Follower sends `NavigateToPose` goals and marks success early at ≤0.20 m; Nav2 tolerance is 0.30 m, keeping movement fluid.

## Data & Logging Flow
```
Gazebo -> /odom -------------------------------> blockchain_logger
AMCL -> /amcl_pose -> waypoint follower -> Nav2 action server
waypoint follower -> /nav2_status -> blockchain_logger
blockchain_logger -> Ganache (transactions)
```

## Determinism
- Static world and fixed waypoint list.
- Deterministic blockchain (`--deterministic` Ganache) preserves ordered event history.


## Extension Ideas
- Add IMU / LIDAR plugins to URDF and feed richer sensor fusion.
- Replace static map->odom with EKF-published transform and integrate GPS or other global inputs.
- Introduce dynamic obstacles to exercise recovery behaviors.
- Remove RViz + gnome-terminal usage and launch the same nodes directly (single shell or TMUX) for CI or remote server contexts.
