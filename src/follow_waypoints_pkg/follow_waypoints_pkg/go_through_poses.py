"""Minimal NavigateThroughPoses launcher.

Loads waypoints from CSV and sends them once to the Nav2 NavigateThroughPoses
action server. Exits when the result is received (success or failure).

CSV expected at: <workspace>/src/follow_waypoints_pkg/resource/odom_waypoints.csv
Columns: x,y,z,qw
Only orientation.w is used (x,y,z set to 0) for simple planar navigation.
"""

import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


class SimpleThroughPoses(Node):
    def __init__(self):
        super().__init__('simple_go_through_poses')
        self._client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._waypoints = self._load_waypoints()

    def _load_waypoints(self):
        path = os.path.join(os.getcwd(), 'src', 'follow_waypoints_pkg', 'resource', 'odom_waypoints.csv')
        waypoints = []
        try:
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(row['x'])
                    pose.pose.position.y = float(row['y'])
                    pose.pose.position.z = float(row['z'])
                    pose.pose.orientation.w = float(row['qw'])
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = 0.0
                    waypoints.append(pose)
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from CSV.')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
        return waypoints

    def run(self):
        if not self._waypoints:
            self.get_logger().error('No waypoints to send. Exiting.')
            return False

        self.get_logger().info('Waiting for NavigateThroughPoses action server...')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('NavigateThroughPoses action server not available.')
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = self._waypoints
        self.get_logger().info(f'Sending {len(goal.poses)} waypoints.')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status = getattr(result, 'status', None)
        if status == 4:  # SUCCEEDED
            self.get_logger().info('NavigateThroughPoses succeeded.')
            return True
        self.get_logger().warn(f'NavigateThroughPoses finished with status code {status}.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = SimpleThroughPoses()
    try:
        success = node.run()
        if not success:
            node.get_logger().warn('Navigation did not succeed.')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
