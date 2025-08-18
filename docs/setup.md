# Setup

Fast path:
```bash
./setup.sh
```

What you need globally:
- ROS 2 Humble Desktop
- colcon
- Node.js 18 + ganache-cli
- Python web3

Manual (if you prefer):
```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-pip
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -

sudo npm i -g ganache-cli
sudo pip3 install web3
```

Just run and enjoy:
```bash
./run.sh
```

Logs decode into `records/decoded_blockchain_logs.json`.

If nothing shows up, wait for the robot to move a little.
