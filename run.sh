#!/bin/bash

echo "Running the Technical Challenge..."

pkill -9 gzserver
pkill -9 gzclient
pkill -9 rviz2
pkill -9 rviz
pkill -9 "ros2 launch"
pkill -9 "ros2 run"
pkill -9 gazebo

rm -r -f build log install

colcon build

# Minimal Ganache prep: ensure dir exists & drop stale LOCK if present
GANACHE_DB_DIR="./blockchain_data"
mkdir -p "$GANACHE_DB_DIR"
pkill -f ganache-cli 2>/dev/null || true
[ -f "$GANACHE_DB_DIR/LOCK" ] && rm -f "$GANACHE_DB_DIR/LOCK"

# terminal 1:
gnome-terminal -- bash -c '
NVM_DIR="$HOME/.nvm"; [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" 2>/dev/null
command -v nvm >/dev/null && nvm use 18 >/dev/null 2>&1 || true

if command -v ganache-cli >/dev/null 2>&1; then
	GANACHE_RUN="ganache-cli"
else
	GANACHE_RUN="npx ganache-cli"
fi

$GANACHE_RUN \
	--port 8545 --deterministic --networkId 1337 \
	--gasLimit 10000000 --gasPrice 20000000000 \
	--accounts 10 --defaultBalanceEther 100 \
	--blockTime 15 --db "$GANACHE_DB_DIR"; exec bash
'

sleep 2

# terminal 2:
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch sdnt_robot_simulation simulation.launch.py; exec bash" & 

sleep 6

python Tools/blockchain_logger.py &

# terminal 3:
gnome-terminal -- bash -c "source install/setup.bash && ros2 run follow_waypoints_pkg odom_follower; exec bash" &
