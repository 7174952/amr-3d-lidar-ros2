import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

class GPSVisualizer(Node):
    def __init__(self):
        super().__init__('gps_visualizer')
        self.sub = self.create_subscription(Odometry, '/odometry/gps', self.callback, 10)
        self.pub_path = self.create_publisher(Path, '/gps_path', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/gps_marker', 10)
        self.path = Path()
        self.path.header.frame_id = "map"

    def callback(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.path)

        m = Marker()
        m.header = msg.header
        m.ns = "gps"
        m.id = len(self.path.poses)
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = ps.pose
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        self.pub_marker.publish(MarkerArray(markers=[m]))

def main(args=None):
    rclpy.init(args=args)
    node = GPSVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

