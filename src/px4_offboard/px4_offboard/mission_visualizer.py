"""
PX4 Mission Visualizer

Subscribes to PX4 state and publishes RViz-friendly visualizations:
- TF transform map -> x500 (drone pose)
- Waypoint markers (planned mission)
- Trajectory path (actual flown path, grows over time)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from tf2_ros import TransformBroadcaster

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


class MissionVisualizer(Node):

    WAYPOINTS = [
        (0.0, 0.0, -5.0),
        (5.0, 0.0, -5.0),
        (5.0, 5.0, -5.0),
        (0.0, 5.0, -5.0),
        (0.0, 0.0, -5.0),
    ]

    WORLD_FRAME = 'map'
    DRONE_FRAME = 'x500'

    def __init__(self):
        super().__init__('mission_visualizer')

        # PX4 uses BEST_EFFORT QoS so subscribers must match (only parameter here that needs to match)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers - PX4 state
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_cb, px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, px4_qos)

        # Publishers - RViz uses default RELIABLE QoS, no need to match PX4
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, '/mission/waypoints', 10)
        self.trajectory_pub = self.create_publisher(
            Path, '/mission/trajectory', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.local_position = VehicleLocalPosition()
        self.attitude = VehicleAttitude()
        self.trajectory_msg = Path()
        self.trajectory_msg.header.frame_id = self.WORLD_FRAME

        # Timers
        # Drone pose updates at 20Hz - smooth visualization
        self.create_timer(0.05, self.publish_drone_tf)
        # Trajectory grows at 5Hz 
        self.create_timer(0.2, self.append_trajectory_point)
        # Waypoint markers republish at 1Hz - cheap, keeps RViz consistent
        self.create_timer(1.0, self.publish_waypoint_markers)

    # --- Callbacks ---
    def local_position_cb(self, msg):
        self.local_position = msg

    def attitude_cb(self, msg):
        self.attitude = msg

    # --- TF: drone pose in world frame ---
    def publish_drone_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.WORLD_FRAME
        t.child_frame_id = self.DRONE_FRAME

        t.transform.translation.x = float(self.local_position.x)
        t.transform.translation.y = float(self.local_position.y)
        t.transform.translation.z = float(self.local_position.z)

        # PX4 attitude is quaternion [w, x, y, z], ROS2 expects [x, y, z, w]
        q = self.attitude.q
        t.transform.rotation.w = float(q[0])
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])

        self.tf_broadcaster.sendTransform(t)

    # --- Trajectory: growing path of actual positions ---
    def append_trajectory_point(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.WORLD_FRAME
        pose.pose.position.x = float(self.local_position.x)
        pose.pose.position.y = float(self.local_position.y)
        pose.pose.position.z = float(self.local_position.z)
        pose.pose.orientation.w = 1.0  # identity quaternion

        self.trajectory_msg.poses.append(pose)
        self.trajectory_msg.header.stamp = pose.header.stamp
        self.trajectory_pub.publish(self.trajectory_msg)

    # --- Waypoint markers: spheres + connecting line ---
    def publish_waypoint_markers(self):
        marker_array = MarkerArray()

        # Sphere at each waypoint
        for i, (x, y, z) in enumerate(self.WAYPOINTS):
            sphere = Marker()
            sphere.header.frame_id = self.WORLD_FRAME
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.5
            sphere.scale.y = 0.5
            sphere.scale.z = 0.5
            sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(sphere)

        # Line strip connecting waypoints in order
        line = Marker()
        line.header.frame_id = self.WORLD_FRAME
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'waypoints'
        line.id = len(self.WAYPOINTS)  # unique id past the spheres
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.1  # line width in meters
        line.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        for x, y, z in self.WAYPOINTS:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            line.points.append(p)
        marker_array.markers.append(line)

        self.waypoint_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
