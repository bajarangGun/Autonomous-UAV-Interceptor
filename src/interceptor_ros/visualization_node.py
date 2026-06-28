"""
ROS 2 3D Trajectory Visualization Node.

Listens to MAVROS telemetry.
Publishes a nav_msgs/Path trail for the interceptor.
Publishes a massive visualization_msgs/Marker (Sphere) for the target.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data

class TrajectoryVisualizer(Node):
    """Generates real-time 3D visuals for RViz2."""

    def __init__(self) -> None:
        super().__init__("trajectory_visualizer")
        
        # Setup Path object to store the Interceptor's trail
        self.int_path = Path()
        self.int_path.header.frame_id = "map"  # RViz standard global frame
        self.max_trail_length = 5000  # Limits trail length to prevent memory overload

        # --- RViz2 PUBLISHERS ---
        self.int_path_pub = self.create_publisher(Path, "/visuals/interceptor_path", 10)
        self.tgt_marker_pub = self.create_publisher(Marker, "/visuals/target_shape", 10)
        
        # --- MAVROS SUBSCRIBERS ---
        self.create_subscription(PoseStamped, "/drone0/local_position/pose", self.int_pose_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/drone1/local_position/pose", self.tgt_pose_cb, qos_profile_sensor_data)
        
        self.get_logger().info("3D Visualizer Node initialized. Ready for RViz2.")

    def int_pose_cb(self, msg: PoseStamped) -> None:
        """Appends Interceptor position to its trail and publishes."""
        msg.header.frame_id = "map"
        self.int_path.poses.append(msg)
        self.int_path.poses = self.int_path.poses[-self.max_trail_length:]
        
        self.int_path.header.stamp = self.get_clock().now().to_msg()
        self.int_path_pub.publish(self.int_path)

    def tgt_pose_cb(self, msg: PoseStamped) -> None:
        """Publishes the Target as a giant 3D shape."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE  # Make it a 3D Sphere
        marker.action = Marker.ADD
        marker.pose = msg.pose
        
        # Scale: Make it a massive 15-meter wide sphere so it's easy to see!
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0
        
        # Color: Bright Solid Red
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (1.0 is fully opaque)
        
        self.tgt_marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
