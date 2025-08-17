import rclpy
import os, csv
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

CSV_RELATIVE_PATH = 'src/follow_waypoints_pkg/resource/odom_waypoints.csv'

class OdomWaypointNavigator(Node):
    """Minimal node: load CSV waypoints, send sequentially, then exit."""
    def __init__(self):
        super().__init__('odom_waypoint_navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = self._load_waypoints()
        # Current robot position (x,y) updated from localization
        self._xy = None
        self._position_tolerance = 0.20  # meters
        # Subscribe to amcl pose (change to '/odom' if needed)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10)

    def _load_waypoints(self):
        waypoints = []
        csv_path = os.path.join(os.getcwd(), CSV_RELATIVE_PATH)
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    z = float(row.get('z', 0.0))
                    qw = float(row.get('qw', 1.0))  # orientation ignored logically
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = z
                    pose.pose.orientation.w = qw
                    waypoints.append(pose)
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
        return waypoints

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available.')
            return
        for i, wp in enumerate(self.waypoints, start=1):
            goal = NavigateToPose.Goal()
            wp.header.stamp = self.get_clock().now().to_msg()
            goal.pose = wp
            self.get_logger().info(f'Sending waypoint {i}/{len(self.waypoints)}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}) (ignoring heading)')
            send_future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            handle = send_future.result()
            if not handle or not handle.accepted:
                self.get_logger().error(f'Waypoint {i} rejected, stopping.')
                return
            result_future = handle.get_result_async()
            # Monitor distance; advance early once within tolerance
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)
                # If goal finished normally, accept and move on
                if result_future.done():
                    self.get_logger().info(f'Waypoint {i} reached (action success).')
                    break
                if self._xy is not None:
                    dx = wp.pose.position.x - self._xy[0]
                    dy = wp.pose.position.y - self._xy[1]
                    dist = (dx*dx + dy*dy) ** 0.5
                    if dist <= self._position_tolerance:
                        self.get_logger().info(f'Waypoint {i} position reached within {dist:.3f} m (<= {self._position_tolerance} m). Cancelling goal and proceeding.')
                        try:
                            handle.cancel_goal_async()
                        except Exception:
                            pass
                        # small grace period
                        for _ in range(5):
                            rclpy.spin_once(self, timeout_sec=0.05)
                        break
        self.get_logger().info('All waypoints completed. Node will shut down.')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self._xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def main(args=None):
    rclpy.init(args=args)
    node = OdomWaypointNavigator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()