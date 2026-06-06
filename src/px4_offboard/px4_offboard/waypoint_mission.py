"""
Waypoint Mission

Flies a square pattern at 5m altitude, then lands.

PX4 uses NED frame (North-East-Down):
  +X = North, +Y = East, +Z = Down (so z = -5.0 means 5 meters up)
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class MissionState(Enum):
    WARMUP = auto()       # Stream heartbeat + setpoint before mode/arm
    ARMING = auto()       # Mode switch + arm sent, waiting for confirmation
    NAVIGATING = auto()   # Visiting waypoints one by one
    LANDING = auto()      # Land command sent, drone descending
    DONE = auto()         # auto() for assigning uniquie values 


class WaypointMission(Node):
    # Square pattern at 5m altitude (z = -5 in NED)
    WAYPOINTS = [
        (0.0,  0.0, -5.0),   # takeoff position
        (5.0,  0.0, -5.0),   # north 5m
        (5.0,  5.0, -5.0),   # east 5m
        (0.0,  5.0, -5.0),   # south back
        (0.0,  0.0, -5.0),   # return home
    ]
    WAYPOINT_TOLERANCE = 0.5  # meters - "close enough" radius
    WARMUP_TICKS = 10          # 1 second of heartbeat before arming

    def __init__(self):
        super().__init__('waypoint_mission')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_cb, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_cb, qos)

        # State
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.state = MissionState.WARMUP
        self.tick = 0
        self.current_wp_index = 0

        self.create_timer(0.1, self.loop)  # 10Hz

    # --- Callbacks ---
    def local_position_cb(self, msg):
        self.local_position = msg

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    # --- Publishers ---
    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp_us()
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = self._timestamp_us()
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self.vehicle_cmd_pub.publish(msg)

    # --- High-level commands ---
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0, param2=21196.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    # --- Helpers ---
    def distance_to(self, x: float, y: float, z: float) -> float:
        dx = self.local_position.x - x
        dy = self.local_position.y - y
        dz = self.local_position.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    # --- Main state machine ---
    def loop(self):
        # ALWAYS stream heartbeat + setpoint, regardless of state
        self.publish_offboard_heartbeat()
        target = self.WAYPOINTS[self.current_wp_index]
        self.publish_position_setpoint(*target)

        if self.state == MissionState.WARMUP:
            if self.tick >= self.WARMUP_TICKS:
                self.engage_offboard_mode()
                self.arm()
                self.state = MissionState.ARMING

        elif self.state == MissionState.ARMING:
            # Wait until PX4 confirms armed + offboard
            if (self.vehicle_status.arming_state == 2 and
                self.vehicle_status.nav_state == 14):
                self.get_logger().info('Armed and in offboard mode, beginning mission')
                self.state = MissionState.NAVIGATING

        elif self.state == MissionState.NAVIGATING:
            dist = self.distance_to(*target)
            if dist < self.WAYPOINT_TOLERANCE:
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_index}: '
                    f'({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})'
                )
                if self.current_wp_index < len(self.WAYPOINTS) - 1:
                    self.current_wp_index += 1
                else:
                    self.get_logger().info('All waypoints reached, landing')
                    self.land()
                    self.state = MissionState.LANDING

        elif self.state == MissionState.LANDING:
            # Wait until landed (z close to 0 and disarmed)
            if self.vehicle_status.arming_state == 1:  # disarmed
                self.get_logger().info('Mission complete')
                self.state = MissionState.DONE

        # Log every second so position is known on a constant basis
        if self.tick % 10 == 0:
            self.get_logger().info(
                f'state={self.state.name}  '
                f'wp={self.current_wp_index}  '
                f'pos=({self.local_position.x:.2f}, '
                f'{self.local_position.y:.2f}, '
                f'{self.local_position.z:.2f})  '
                f'target=({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})'
            )

        self.tick += 1

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
