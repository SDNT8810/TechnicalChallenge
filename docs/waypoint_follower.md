# Waypoint Follower

`follow_waypoints_pkg` provides three lightweight Python nodes for different waypoint delivery patterns to Nav2.

## Components
| File | Purpose | Action Interface | Source of Waypoints |
|------|---------|------------------|---------------------|
| `odom_follower.py` | Sequential waypoint navigation with live distance monitoring + status publishing | `navigate_to_pose` (NavigateToPose) | `resource/odom_waypoints.csv` (x,y,z,qw) |
| `go_through_poses.py` | Batch navigation through a list (single multi-pose goal) | `navigate_through_poses` (NavigateThroughPoses) | `resource/odom_waypoints.csv` (x,y,z,qw) |
| `gps_follower.py` | Sequential navigation using simple lat/long to map scaling | `navigate_to_pose` (NavigateToPose) | `resource/gps_waypoints.csv` (latitude,longitude) |

## CSV Expectations
- Odometry file (`odom_waypoints.csv`): headers `x,y,z,qw`. Orientation x,y,z assumed 0; only w retained. Frame id: `map`.
- GPS file (`gps_waypoints.csv`): headers `latitude,longitude`. Converted to map frame with fixed divisors (lat/−30 → x, lon/−10 → y). Orientation fixed (w=1).

## odom_follower.py
Flow:
1. Load waypoints at startup (absolute path via CWD + relative CSV path).
2. Connect to `navigate_to_pose` action server (timeout 10 s).
3. Subscribe to `/amcl_pose` for current pose; store `(x,y)`.
4. Publish status changes on `/nav2_status` (String). States used: `waitin`, `get target`, `moving`, `reached target`.
5. For each waypoint:
   - Send a NavigateToPose goal.
   - Loop at 10 Hz: spin, update status to `moving`, check if action done.
   - Early success: if Euclidean distance ≤ 0.20 m, cancel goal, proceed.
   - On last waypoint completion set `reached target` else revert to `waitin`.
6. Exit after final waypoint.

Key behaviors:
- Early cancellation trims latency between waypoints.
- Status topic feeds blockchain logger without coupling.

## go_through_poses.py
Single multi-pose goal:
1. Load full list (same CSV as odom_follower).
2. Wait (15 s timeout) for `navigate_through_poses` server.
3. Send one goal containing all `PoseStamped` entries.
4. Block until result. Success status code 4 logged; non‑success reported and the node exits.

Use case: lets Nav2 planner optimize across the sequence internally.

## gps_follower.py
Sequential approach similar to `odom_follower` but simpler:
1. Load GPS CSV into list of `[latitude, longitude]` pairs.
2. Transform each to a `PoseStamped` using fixed scaling (no projection, approximate placeholder).
3. Send waypoints one at a time via `navigate_to_pose`.
4. Wait synchronously for each action result; no mid-goal distance monitoring or status publisher.

Intended as a sketch for integrating geodetic inputs; not performing proper geodesy or frame transforms.

## Design Choices
- Separation between batch (`NavigateThroughPoses`) vs iterative (`NavigateToPose`) keeps logic explicit and easy to test.
- CSV parsing with `csv.DictReader` avoids positional assumptions and keeps files editable by hand.
- Minimal dependencies: only core ROS 2 messages and Nav2 action types (see package.xml exec_depend list).
- Status publishing confined to the odometry sequential variant to avoid noise; other nodes stay silent.

## Limitations
- No dynamic reconfiguration or live waypoint injection.
- GPS conversion is placeholder; real deployment needs proper map projection and TF handling.
- No recovery or retry logic on aborted goals beyond logging.
- Orientation ignored except for scalar w, so heading control is coarse.

## Running (examples)
```bash
# Sequential odometry waypoints with status publishing
ros2 run follow_waypoints_pkg odom_follower

# Batch multi-pose navigation
ros2 run follow_waypoints_pkg go_through_poses

# GPS-based sequential navigation (prototype scaling)
ros2 run follow_waypoints_pkg gps_follower
```
Ensure Nav2 stack and localization are active before launching any follower node.
