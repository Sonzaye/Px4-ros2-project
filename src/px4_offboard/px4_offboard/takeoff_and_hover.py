"""
PX4 Offboard Control: Takeoff and Hover

Arms the drone, switches to offboard mode, takes off to 5m altitude,
hovers for 20 seconds, then lands.

PX4 uses NED frame (North-East-Down):
  +X = North, +Y = East, +Z = Down
  So z = -5.0 means 5 meters above ground.
"""

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


class TakeoffAndHover(Node):
    def __init__(self):
        super().__init__('takeoff_and_hover')

        # PX4 uses BEST_EFFORT + TRANSIENT_LOCAL QoS over uXRCE-DDS.
        # Mismatched QoS = silently no messages flow.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers - command channels to PX4
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers - state from PX4
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_cb, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_cb, qos)

        # State
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.tick = 0
        self.takeoff_altitude = -5.0  # NED: negative = up

        # Main loop at 10Hz - publishes heartbeat and runs state machine
        self.create_timer(0.1, self.loop)

    # --- Callbacks ---
    def local_position_cb(self, msg):
        self.local_position = msg

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    # --- Publishers ---
    def publish_offboard_heartbeat(self):
        """Tell PX4 we're commanding position. Must publish at >2Hz."""
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
        msg.yaw = 0.0  # facing north
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
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)
        self.get_logger().info('Arm command sent (force)')

    def engage_offboard_mode(self):
        # DO_SET_MODE with param1=1 (custom mode) param2=6 (offboard)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Engage offboard mode command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    # --- Main state machine ---
    def loop(self):
        # Both heartbeat AND setpoint must stream continuously for offboard.
        # PX4 won't arm in offboard mode unless it sees a setpoint stream.
        self.publish_offboard_heartbeat()
        self.publish_position_setpoint(0.0, 0.0, self.takeoff_altitude)

        # After 1 second of streaming, switch mode and arm
        if self.tick == 10:
            self.engage_offboard_mode()
            self.arm()

        # After 21 seconds total, land
        if self.tick == 210:
            self.land()

        # Log every second
        if self.tick % 10 == 0:
            self.get_logger().info(
                f'tick={self.tick}  '
                f'z={self.local_position.z:.2f}m  '
                f'nav_state={self.vehicle_status.nav_state}  '
                f'arming_state={self.vehicle_status.arming_state}'
            )

        self.tick += 1

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffAndHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
