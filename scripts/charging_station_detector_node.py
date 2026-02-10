#!/usr/bin/env python3

"""
Charging Station Detector Node
ค้นหาและนำทางไปยัง charging station ในแผนที่

Features:
- Detect charging station location from map
- Integrate with Nav2 for global navigation
- Monitor charging station availability
- Provide closest charging station recommendation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
import os


class ChargingStation:
    """Data class for charging station"""
    def __init__(self, station_id: str, pose: Pose, name: str = "", occupied: bool = False):
        self.station_id = station_id
        self.pose = pose
        self.name = name if name else f"Station_{station_id}"
        self.occupied = occupied
        self.last_used = None
    
    def distance_to(self, point: Point) -> float:
        """Calculate distance to a point"""
        dx = self.pose.position.x - point.x
        dy = self.pose.position.y - point.y
        return math.sqrt(dx*dx + dy*dy)


class ChargingStationDetector(Node):
    """
    Charging Station Detector and Navigator
    """
    
    def __init__(self):
        super().__init__('charging_station_detector_node')
        
        # Parameters
        self.declare_parameter('charging_stations_file', '')
        self.declare_parameter('auto_create_stations', False)
        self.declare_parameter('marker_detection_enabled', True)
        
        stations_file = self.get_parameter('charging_stations_file').value
        self.auto_create = self.get_parameter('auto_create_stations').value
        self.marker_enabled = self.get_parameter('marker_detection_enabled').value
        
        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.sub_go_charge = self.create_subscription(
            String,
            '/charging/goto_station',
            self.goto_station_callback,
            10)
        
        self.sub_should_charge = self.create_subscription(
            Bool,
            '/battery/should_charge',
            self.should_charge_callback,
            10)
        
        # Publishers
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/charging_stations/markers',
            10)
        
        self.pub_closest_station = self.create_publisher(
            PoseStamped,
            '/charging_stations/closest',
            10)
        
        self.pub_status = self.create_publisher(
            String,
            '/charging_stations/status',
            10)
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.charging_stations = {}  # Dict[station_id, ChargingStation]
        self.current_position = None
        self.currently_navigating = False
        self.target_station_id = None
        
        # Load or create charging stations
        if stations_file and os.path.exists(stations_file):
            self.load_stations_from_file(stations_file)
        elif self.auto_create:
            self.create_default_stations()
        else:
            self.get_logger().warn('No charging stations configured. Use charging_stations_file parameter.')
        
        # Timer for periodic tasks
        self.create_timer(2.0, self.publish_markers_callback)
        self.create_timer(5.0, self.update_closest_station)
        
        self.get_logger().info(f'Charging Station Detector Started with {len(self.charging_stations)} stations')

    def load_stations_from_file(self, filepath: str):
        """Load charging station locations from YAML file"""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            stations = data.get('charging_stations', [])
            
            for station_data in stations:
                station_id = station_data['id']
                name = station_data.get('name', f'Station_{station_id}')
                
                pose = Pose()
                pose.position.x = float(station_data['position']['x'])
                pose.position.y = float(station_data['position']['y'])
                pose.position.z = float(station_data['position'].get('z', 0.0))
                
                # Orientation (quaternion)
                orientation = station_data.get('orientation', {'x': 0, 'y': 0, 'z': 0, 'w': 1})
                pose.orientation.x = float(orientation.get('x', 0.0))
                pose.orientation.y = float(orientation.get('y', 0.0))
                pose.orientation.z = float(orientation.get('z', 0.0))
                pose.orientation.w = float(orientation.get('w', 1.0))
                
                station = ChargingStation(station_id, pose, name)
                self.charging_stations[station_id] = station
            
            self.get_logger().info(f'Loaded {len(self.charging_stations)} charging stations from {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load charging stations: {str(e)}')

    def create_default_stations(self):
        """Create default charging stations for testing"""
        # Station 1: At origin
        pose1 = Pose()
        pose1.position.x = 0.0
        pose1.position.y = 0.0
        pose1.position.z = 0.0
        pose1.orientation.w = 1.0
        
        station1 = ChargingStation('station_1', pose1, 'Main Charging Station')
        self.charging_stations['station_1'] = station1
        
        self.get_logger().info('Created default charging station at origin')

    def save_stations_to_file(self, filepath: str):
        """Save current charging stations to YAML file"""
        stations_data = []
        
        for station_id, station in self.charging_stations.items():
            station_dict = {
                'id': station_id,
                'name': station.name,
                'position': {
                    'x': station.pose.position.x,
                    'y': station.pose.position.y,
                    'z': station.pose.position.z
                },
                'orientation': {
                    'x': station.pose.orientation.x,
                    'y': station.pose.orientation.y,
                    'z': station.pose.orientation.z,
                    'w': station.pose.orientation.w
                }
            }
            stations_data.append(station_dict)
        
        data = {'charging_stations': stations_data}
        
        try:
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f'Saved {len(stations_data)} charging stations to {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save charging stations: {str(e)}')

    def odom_callback(self, msg: Odometry):
        """Update current position"""
        self.current_position = msg.pose.pose.position

    def should_charge_callback(self, msg: Bool):
        """Handle battery low signal - automatically navigate to charging"""
        if msg.data and not self.currently_navigating:
            self.get_logger().warn('Battery predictor recommends charging. Finding closest station...')
            self.navigate_to_closest_station()

    def goto_station_callback(self, msg: String):
        """Handle manual charging station navigation request"""
        station_id = msg.data
        
        if station_id == "closest":
            self.navigate_to_closest_station()
        elif station_id in self.charging_stations:
            self.navigate_to_station(station_id)
        else:
            self.get_logger().error(f'Unknown charging station: {station_id}')

    def find_closest_station(self) -> str:
        """
        Find the closest available charging station
        Returns: station_id or None
        """
        if not self.current_position or not self.charging_stations:
            return None
        
        closest_id = None
        min_distance = float('inf')
        
        for station_id, station in self.charging_stations.items():
            if station.occupied:
                continue  # Skip occupied stations
            
            distance = station.distance_to(self.current_position)
            
            if distance < min_distance:
                min_distance = distance
                closest_id = station_id
        
        return closest_id

    def navigate_to_closest_station(self):
        """Navigate to the closest available charging station"""
        closest_id = self.find_closest_station()
        
        if closest_id:
            self.navigate_to_station(closest_id)
        else:
            self.get_logger().error('No available charging stations found!')

    def navigate_to_station(self, station_id: str):
        """
        Send navigation goal to specific charging station
        Args:
            station_id: ID of the charging station
        """
        if station_id not in self.charging_stations:
            self.get_logger().error(f'Station {station_id} not found!')
            return
        
        station = self.charging_stations[station_id]
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = station.pose
        
        self.get_logger().info(f'Navigating to {station.name} at ({station.pose.position.x:.2f}, {station.pose.position.y:.2f})')
        
        # Send goal
        self.currently_navigating = True
        self.target_station_id = station_id
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.currently_navigating = False
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback"""
        # Can log progress here if needed
        pass

    def nav_result_callback(self, future):
        """Handle Nav2 result"""
        result = future.result().result
        self.currently_navigating = False
        
        if result:
            self.get_logger().info('Navigation to charging station completed!')
            # Publish status
            status_msg = f'Arrived at {self.charging_stations[self.target_station_id].name}'
            self.pub_status.publish(String(data=status_msg))
        else:
            self.get_logger().error('Navigation to charging station failed!')

    def update_closest_station(self):
        """Update and publish closest station info"""
        if not self.current_position:
            return
        
        closest_id = self.find_closest_station()
        
        if closest_id:
            station = self.charging_stations[closest_id]
            
            # Publish as PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose = station.pose
            
            self.pub_closest_station.publish(pose_msg)

    def publish_markers_callback(self):
        """Publish visualization markers for all charging stations"""
        marker_array = MarkerArray()
        
        for i, (station_id, station) in enumerate(self.charging_stations.items()):
            # Station marker (cylinder)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'charging_stations'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose = station.pose
            marker.pose.position.z = 0.5  # Raise it up
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.0
            
            # Color: green if available, red if occupied
            marker.color.a = 0.8
            if station.occupied:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'charging_station_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = station.pose
            text_marker.pose.position.z = 1.5
            
            text_marker.text = station.name
            text_marker.scale.z = 0.3
            
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ChargingStationDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
