# Technical Challenge Report

This report maps the original challenge requirements in `TechnicalChallenge.txt` to the implemented system present in this repository. All details below come from the current codebase (no speculation).

## 1. Goal & Scenario
Chosen goal: autonomously visit three fixed waypoints on a provided map using Nav2.

Waypoints file: `src/follow_waypoints_pkg/resource/odom_waypoints.csv`
```
(-2.0, -1.0)
( 4.0, -1.8)
( 3.0,  1.7)
```
Frame: `map`.

## 2. ROS Functionality
Component | Implementation
----------|----------------
Simulation | Gazebo world: `src/sdnt_robot_simulation/world/warehouse.world`
Map | `src/sdnt_robot_simulation/maps/warehouse.yaml` (+ pgm)
Robot Description | URDF: `src/sdnt_robot_simulation/src/description/sdnt_robot_description.urdf`
Launch | `simulation.launch.py` starts gzserver, robot_state_publisher, joint_state_publisher, ekf (`robot_localization`), map_server, AMCL, Nav2, static TF map->odom, RViz2
Navigation | Nav2 bringup via `navigation_launch.py` (included) and `nav2_params.yaml`
Localization | EKF + AMCL nodes
Waypoint Execution | `odom_follower.py` (NavigateToPose action client) sequentially sends waypoints
Status Publication | `/nav2_status` (std_msgs/String) values: `waitin`, `get target`, `moving`, `reached target`

## 3. Data Captured
Source | Fields
-------|-------
Odometry (`/odom`) | x, y (extracted by logger each second if moved > 0.1 m)
Status (`/nav2_status`) | Discrete string states (above)
Timestamp | Wall clock `time.time()` for each log event

Logger file: `Tools/blockchain_logger.py`.
Trigger conditions: movement distance > 0.1 m since last logged OR status change.
Data object structure:
```json
{"timestamp": <float>, "robot": {"time": <float>, "x": <x>, "y": <y>, "status": "<string>"}}
```
Numeric formatting: x,y rounded to 3 decimals in blockchain transaction.
Logging interval timer: 1.0 s.

Fallback: if blockchain unavailable, append to `records/movement_log.json`.

## 4. Blockchain Integration
Local chain: Ganache CLI.
Launch parameters (from `run.sh`):
```
--port 8545
--deterministic
--networkId 1337
--gasLimit 10000000
--gasPrice 20000000000
--accounts 10
--defaultBalanceEther 100
--blockTime 15
--db blockchain_data/
```
Account used: first Ganache account (`accounts[0]`) as both `from` and `to` (self transaction).
Payload: UTF-8 JSON hex encoded (prefixed `0x`).
Gas estimation formula in code: `21000 + (len(data_hex) - 2) * 16`.

Extractor: `Tools/extract_blockchain_data.py`.
Process:
1. Connect to Ganache (HTTP 8545).
2. Get latest block number.
3. Iterate blocks 0..latest.
4. For each tx where `from == to == accounts[0]` and `input` length > 10, decode `tx.input` bytes -> JSON.
5. Collect enriched records: block_number, transaction_hash, timestamp, human_time, robot{...}.
6. Sort by timestamp; write `records/decoded_blockchain_logs.json`.
7. Print summary (first/last record times, total count, start/end position).

## 5. Experiment Methodology
Sequence (from `run.sh`):
1. Kill previous gazebo / rviz / ros2 processes.
2. Remove `build/ log/ install/`.
3. Build with `colcon build`.
4. Start Ganache (new terminal, persistent DB `blockchain_data/`).
5. Launch simulation (Nav2, localization, RViz).
6. Start blockchain logger (background Python process).
7. Start waypoint follower node (new terminal) – publishes statuses and sends goals.
8. After delay, run extractor script to decode existing transactions.

Expected behavior: robot moves through 3 waypoints; status transitions and position changes produce blockchain transactions.

## 6. Visualization
Tool: RViz2 launched by `simulation.launch.py` with config `rviz/display.rviz` (present in package). Provides map, robot pose, navigation visualization. No separate plotting script included; visualization criterion satisfied via RViz session.

## 7. Reproducibility
Single command orchestration: `./run.sh` after environment setup (`./setup.sh`). Deterministic Ganache (`--deterministic`) and fixed waypoint CSV ensure repeatable run order and chain data sequence (aside from timestamps).

## 8. Possible Improvements (Grounded in Current Code)
Category | Improvement
---------|------------
Blockchain | Use contract events instead of self transactions; compress payload; include orientation
Navigation | Add recovery behaviors or loop mode after last waypoint
Data | Add orientation, velocity, and action result codes to payload
Visualization | Add lightweight plotting of decoded log (position vs time) script
Robustness | Replace sleep-based sequencing with readiness checks (wait for /navigate_to_pose action server before launching follower)
Headless | Provide headless launch (no RViz, no gnome-terminal) for CI

## 9. Mapping Requirements to Implementation
Requirement (from brief) | Implemented Element
-------------------------|-------------------
Simulate mobile robot (ROS 2 + Gazebo) | `simulation.launch.py`, Gazebo world, URDF
Autonomous navigation on pre-defined map | Nav2 stack + `warehouse.yaml` map + waypoint follower
Record timestamps, position, status | `blockchain_logger.py` JSON structure
Define single clear goal (e.g., three waypoints) | 3 waypoints in CSV file
Methodology / expected behavior | Documented sequence in this report & `run.sh`
Visualization of behavior | RViz2 with Nav2 components
Ganache local blockchain | Ganache invoked with params in `run.sh`
web3.py script sending tx | `blockchain_logger.py` (send_transaction)
Include timestamp, position, status in tx | JSON payload fields
Store data as transaction payload | Hex-encoded JSON `data` field
Brief experiment description | README + this report

## 10. Limitations
- No smart contract; uses raw self transactions (simpler but less structured on-chain querying).
- No external plotting of decoded data.
- Status strings are minimal and include a typo (`waitin`).
- Movement threshold only uses planar distance and fixed 0.1 m value.
- Gas estimate is a rough upper bound; not calibrated.

## 11. How to Re-Run
```bash
./setup.sh          # one-time (if needed)
source /opt/ros/humble/setup.bash
./run.sh
# After run: inspect records/decoded_blockchain_logs.json
```

## 12. Produced Artifacts
Path | Description
-----|------------
`blockchain_data/` | Persistent Ganache chain state
`records/decoded_blockchain_logs.json` | Ordered decoded log entries
`records/movement_log.json` | Fallback file logging (only if chain unavailable)

## 13. Conclusion
The repository meets the core challenge goals: autonomous waypoint navigation on a map, real-time logging of position and status, blockchain transaction storage of selected events, and interactive visualization via RViz. Areas for enhancement are documented without altering the delivered minimal implementation.
