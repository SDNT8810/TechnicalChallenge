#!/usr/bin/env bash
set -e

echo "Setting up (global installs)"

# ROS 2 Humble Desktop (skip if already there)
if [ ! -f /opt/ros/humble/setup.bash ]; then
    sudo apt update
    sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release; echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-pip
fi

# Basic tools
sudo apt install -y build-essential git gnome-terminal

# Node.js 18 (replace if different major)
if command -v node >/dev/null 2>&1; then
    if [ "$(node -p 'process.versions.node.split(".")[0]')" != "18" ]; then
        curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
        sudo apt install -y nodejs
    fi
else
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt install -y nodejs
fi

# Ganache CLI
if ! command -v ganache-cli >/dev/null 2>&1; then
    sudo npm install -g ganache-cli
fi

# Python web3
python3 -c "import web3" 2>/dev/null || sudo pip3 install web3

echo "Done. Next: source /opt/ros/humble/setup.bash && ./run.sh"
