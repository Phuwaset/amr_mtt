import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class BoxMarkerPublisher(Node):
    def __init__(self):
        super().__init__('box_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Publishing Target Box Marker to RViz at (-1.5, -0.7, 0.85)')

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Reference frame in RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position of the box in Gazebo (Target Coordinates)
        marker.pose.position.x = -1.5
        marker.pose.position.y = -0.7
        marker.pose.position.z = 0.85 # Assume height of the table + box
        
        # Orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Dimensions of the box
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color (RGBA: Red half transparent)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    box_marker_publisher = BoxMarkerPublisher()
    rclpy.spin(box_marker_publisher)
    box_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
