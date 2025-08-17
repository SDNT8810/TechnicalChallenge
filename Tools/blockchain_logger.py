#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import time
from web3 import Web3
import math

class BlockchainLogger(Node):
    def __init__(self):
        super().__init__('blockchain_logger')
        
        # Connect to Ganache
        try:
            self.w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
            if self.w3.is_connected():
                self.get_logger().info("✅ Connected to Ganache blockchain!")
                # Use the first account
                self.account = self.w3.eth.accounts[0]
                self.get_logger().info(f"Using account: {self.account}")
            else:
                self.get_logger().error("❌ Failed to connect to Ganache")
                self.w3 = None
        except Exception as e:
            self.get_logger().error(f"❌ Blockchain connection error: {e}")
            self.w3 = None

        # Subscribers for robot position & navigation status (String)
        self.odom_sub_robot = self.create_subscription(Odometry, '/odom', self.odom_callback_robot, 10)
        self.status_sub_robot = self.create_subscription(String, '/nav2_status', self.status_callback_robot, 10)

        # Initialize status and last logged position
        self.robot_status = 'unknown'
        self.last_logged_robot = {'x': 0, 'y': 0, 'time': 0, 'status': 'unknown'}

        # Logging timer (every 1 second)
        self.log_timer = self.create_timer(1.0, self.log_positions)

        self.get_logger().info("Blockchain Logger Started - Recording robot movements!")

    def odom_callback_robot(self, msg):
        self.robot_pos = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'time': time.time()
        }
    def status_callback_robot(self, msg: String):
        self.robot_status = msg.data

    def log_positions(self):
        if not hasattr(self, 'robot_pos'):
            return

        status_changed = self.robot_status != self.last_logged_robot.get('status')
        robot_moved = self.has_moved(self.robot_pos, self.last_logged_robot)

        if robot_moved or status_changed:
            self.log_to_blockchain()
            self.last_logged_robot = self.robot_pos.copy()
            self.last_logged_robot['status'] = self.robot_status

    def has_moved(self, current_pos, last_pos):
        # Check if robot moved more than 0.1m
        distance = math.sqrt((current_pos['x'] - last_pos['x'])**2 + 
                           (current_pos['y'] - last_pos['y'])**2)
        return distance > 0.1

    def log_to_blockchain(self):
        if not self.w3:
            self.get_logger().warn("⚠️  Blockchain not available, logging to file")
            self.log_to_file()
            return
            
        try:
            # Create transaction data
            log_data = {
                'timestamp': time.time(),
                'robot': {
                    'time': self.robot_pos['time'],
                    'x': round(self.robot_pos['x'], 3),
                    'y': round(self.robot_pos['y'], 3),
                    'status': self.robot_status
                }
            }
            
            # Convert to hex data
            data_str = json.dumps(log_data)
            data_hex = '0x' + data_str.encode('utf-8').hex()
            
            # Estimate gas needed for data transaction
            gas_needed = 21000 + (len(data_hex) - 2) * 16  # Base gas + data cost
            
            # Send transaction
            tx_hash = self.w3.eth.send_transaction({
                'from': self.account,
                'to': self.account,  # Self-transaction for logging
                'value': 0,
                'data': data_hex,
                'gas': gas_needed
            })
            
            self.get_logger().info(f"🔗 Logged to blockchain: {tx_hash.hex()[:10]}...")
            self.get_logger().info(f"   robot: ({log_data['robot']['x']}, {log_data['robot']['y']})")
            
        except Exception as e:
            self.get_logger().error(f"❌ Blockchain logging error: {e}")
            self.log_to_file()

    def log_to_file(self):
        # Fallback logging to file
        log_data = {
            'timestamp': time.time(),
            'robot': {
                'x': round(self.robot_pos['x'], 3),
                'y': round(self.robot_pos['y'], 3),
                'time': self.robot_pos['time'],
                'status': self.robot_status
            }
        }
        
        with open('/home/sdnt/Desktop/copilot/movement_log.json', 'a') as f:
            f.write(json.dumps(log_data) + '\n')

        self.get_logger().info(f"📁 Logged to file: robot({log_data['robot']['x']}, {log_data['robot']['y']}), Status: {log_data['robot']['status']})")

def main():
    rclpy.init()
    logger = BlockchainLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
