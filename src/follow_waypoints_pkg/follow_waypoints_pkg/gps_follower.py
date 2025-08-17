import rclpy
import os, json, csv
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class SequentialWaypointNavigator(Node):
    def __init__(self):
        super().__init__('sequential_waypoint_navigator')
        self.Shared_Path = os.getcwd()

        # Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # List of waypoints
        self.waypoints = self.create_waypoints()

        # Start navigation
        self.navigate_waypoints()

    def create_waypoints(self):
        """Define the waypoints from the CSV file as a list of PoseStamped."""
        from geometry_msgs.msg import PoseStamped
        import json

        # Load waypoints from the CSV using your load_csv function
        csv_waypoints_json = self.load_csv()  # This will call your load_csv function
        csv_waypoints = json.loads(csv_waypoints_json)  # Deserialize the JSON string to a Python list

        waypoints = []

        # Iterate through the list of waypoints
        for point in csv_waypoints:
            print('Sending Pos: ', point)
            latitude, longitude = point  # Each point is a [latitude, longitude] list
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = latitude/(-30)  # Map latitude to x
            waypoint.pose.position.y = longitude/(-10)  # Map longitude to y
            waypoint.pose.orientation.w = 1.0    # Default orientation
            waypoints.append(waypoint)

        return waypoints

    def navigate_waypoints(self):
        """Send waypoints sequentially to the NavigateToPose action server."""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')

        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f'Sending waypoint {i+1}...')

            goal = NavigateToPose.Goal()
            goal.pose = waypoint

            # Send the goal
            send_goal_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error(f'Waypoint {i+1} goal was rejected by the server.')
                continue

            self.get_logger().info(f'Waypoint {i+1} goal accepted. Waiting for result...')
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result()

            if result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'Waypoint {i+1} reached successfully!')
            else:
                self.get_logger().error(f'Failed to reach waypoint {i+1}. Status: {result.status}')

        self.get_logger().info('All waypoints completed.')

    def load_csv(self):
        import csv
        file_path = self.Shared_Path + "/src/follow_waypoints_pkg/resource/gps_waypoints.csv"

        if file_path:
            waypoints = []
            try:
                with open(file_path, "r") as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        latitude = float(row["latitude"])
                        longitude = float(row["longitude"])
                        waypoints.append([latitude, longitude])  # Ensure this is a list, not a tuple
                print("Parsed Waypoints:", waypoints)
                return json.dumps(waypoints)  # Serialize as JSON
            except Exception as e:
                print(f"Error loading CSV: {e}")
                return json.dumps([])  # Return an empty list if parsing fails
        else:
            return json.dumps([])  # Return an empty list if no file selected

    def goal_response_callback(self, future):
        """Callback for when a goal response is received."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback for when the result of a goal is received."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')

def main(args=None):
    # navigator = BasicNavigator()
    rclpy.init(args=args)
    # navigator.waitUntilNav2Active()
    print("###########################################################################################################")
    time.sleep(1)
    node = SequentialWaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return 'Hello'

if __name__ == '__main__':
    main()
