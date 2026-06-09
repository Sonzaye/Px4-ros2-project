"""
Precision Lander

Detects a colored target via /perception/color_detection and lands on it.

State machine: IDLE -> SEEKING -> CENTERING -> DESCENDING -> LANDING -> DONE
Start: ros2 param set /precision_lander target_color red  (or green/blue)

PX4 uses NED: +X north, +Y east, +Z down. Altitude is -z.
"""

import math
from enum import Enum, auto
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from geometry_msgs.msg import PointStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude
)
from sensor_msgs.msg import CameraInfo


class State(Enum):
    IDLE = auto()
    SEEKING = auto()
    CENTERING = auto()
    DESCENDING = auto()
    LANDING = auto()
    DONE = auto()


class PrecisionLander(Node):
    IMAGE_WIDTH = 1280
    IMAGE_HEIGHT = 960
    HFOV = 1.74

    CENTERING_TOLERANCE = 0.5        # meters; xy "centered" threshold
    DESCEND_ALTITUDE_THRESHOLD = -1.0  # hand off to NAV_LAND below 1m
    SEARCH_ALTITUDE = -5.0            # hover/search at 5m
    DETECTION_TIMEOUT_TICKS = 10      # 10 ticks @ 10Hz = 1.0s

    # Loop runs at this rate; used for tick-based timing.
    LOOP_HZ = 10.0

    NAV_STATE_AUTO_LAND = 18

    def __init__(self):
        super().__init__('precision_lander')

        self.declare_parameter(
            'target_color', '',
            ParameterDescriptor(description='red, green, or blue. Empty = idle.'))
        self.add_on_set_parameters_callback(self.on_param_change)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_cb, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_cb, px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude_q_v4',
            self.attitude_cb, px4_qos)
        self.create_subscription(
            CameraInfo,
            '/world/default/model/x500_mono_cam_0/link/down_camera_link/sensor/down_camera/camera_info',
            self.camera_info_cb, sensor_qos)
        self.create_subscription(
            PointStamped, '/perception/marker_detection',
            self.detection_cb, sensor_qos)

        # State
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.target_color = ''
        self.cx = self.cy = self.fx = self.fy = 0.0

        # Detection: store offset + a tick-based age. No clocks.
        self.last_det_dx = 0.0
        self.last_det_dy = 0.0
        self.last_det_color = ''
        self.detection_age_ticks = 999  # large = stale
        self.centerer = 999
        self.seeking_ticks = 0
        # Commanded target
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = self.SEARCH_ALTITUDE
        self.target_yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0

        # Locked centered position for descent
        self.search_radius = 2.0
        self.search_angle = 0.0
        self.search_center = (0.0, 0.0)
        self.centered_x = None
        self.centered_y = None

        # Search pattern (expanding square around start)
        self.search_waypoints = []
        self.search_index = 0

        self.state = State.IDLE
        self.lock = threading.Lock()

        self.create_timer(1.0 / self.LOOP_HZ, self.loop)
        self.create_timer(1.0, self.print_status)

        self.get_logger().info('Precision lander ready. Set target_color to start.')

    # --- callbacks ---

    def attitude_cb(self, msg):
        # msg.q is [w, x, y, z]
        w, x, y, z = msg.q
        # roll (x-axis) and pitch (y-axis) from quaternion
        self.roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))

    def on_param_change(self, params):
        for p in params:
            if p.name != 'target_color':
                continue
            color = p.value.strip()   # now this holds the marker ID, e.g. "0"
            # accept empty (idle) or any numeric marker id
            if color != '' and not color.isdigit():
                return SetParametersResult(
                    successful=False,
                    reason='target must be a marker ID number or empty')
            with self.lock:
                self.target_color = color
                if color:
                    self.engage_offboard_mode()
                    self.force_arm()
                    sx, sy = self.local_position.x, self.local_position.y
                    self.target_x = sx
                    self.target_y = sy
                    self.target_z = self.SEARCH_ALTITUDE
                    self.centered_x = None
                    self.centered_y = None
                    self.detection_age_ticks = 999
                    self.seeking_ticks = 0
                    # Expanding-square search around the start point
                    step = 2.0
                    self.search_waypoints = [
                        (sx, sy),
                        (sx + step, sy),
                        (sx + step, sy + step),
                        (sx - step, sy + step),
                        (sx - step, sy - step),
                        (sx + 2 * step, sy - step),
                        (sx + 2 * step, sy + 2 * step),
                        (sx - 2 * step, sy + 2 * step),
                        (sx - 2 * step, sy - 2 * step),
                    ]
                    self.search_index = 0
                    self.state = State.SEEKING
                    self.get_logger().info(f'Mission start: hunting {color}')
                else:
                    self.state = State.IDLE
                    self.get_logger().info('Mission cleared')
            return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def local_position_cb(self, msg):
        self.local_position = msg

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    def camera_info_cb(self, msg):
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.fx = msg.k[0]
        self.fy = msg.k[4]

    def detection_cb(self, msg):
        with self.lock:
            if msg.header.frame_id != self.target_color:
                return
            if self.fx == 0:
                return
            altitude = -self.local_position.z
            if altitude < 0.5:
                return
            # The downward camera lies when the drone is tilted: a level cube
            # appears off-center purely because the camera rotated. Only trust
            # detections taken while nearly level, so the estimate is honest.
            LEVEL_TOL = 0.20  # radians 
            if abs(self.roll) > LEVEL_TOL or abs(self.pitch) > LEVEL_TOL:
                return

            du = msg.point.x - self.cx
            dv = msg.point.y - self.cy

            
            # pixel -> body-frame ground offset
            body_dx = -dv * altitude / self.fy
            body_dy =  du * altitude / self.fx
            # body -> world (NED) using yaw
            yaw = self.local_position.heading
            world_dx = body_dx * math.cos(yaw) - body_dy * math.sin(yaw)
            world_dy = body_dx * math.sin(yaw) + body_dy * math.cos(yaw)

            self.last_det_dx = world_dx
            self.last_det_dy = world_dy
            self.last_det_color = msg.header.frame_id
            self.detection_age_ticks = 0  # fresh

    # --- PX4 commands ---

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def force_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0, param2=21196.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # --- publishers ---

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self._timestamp_us()
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        with self.lock:
            msg.position = [self.target_x, self.target_y, self.target_z]
            msg.yaw = self.target_yaw
        msg.timestamp = self._timestamp_us()
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f'param{i}', params.get(f'param{i}', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self.vehicle_cmd_pub.publish(msg)

    # --- main loop ---

    def loop(self):
        if self.vehicle_status.nav_state != self.NAV_STATE_AUTO_LAND:
            self.publish_offboard_heartbeat()
            self.publish_position_setpoint()
        with self.lock:
            self.tick_state_machine()

    def tick_state_machine(self):
        # tick-based freshness
        self.detection_age_ticks += 1
        detection_fresh = (
            self.detection_age_ticks <= self.DETECTION_TIMEOUT_TICKS
            and self.last_det_color == self.target_color
        )
        det_dx, det_dy = self.last_det_dx, self.last_det_dy

        if self.state == State.IDLE:
            return

        if (self.state == State.SEEKING):
            self.target_z = self.SEARCH_ALTITUDE
            if detection_fresh:
                self.state = State.CENTERING
                self.get_logger().info('Target acquired, centering')
                self.seeking_ticks = 0
                return
            # drive the search pattern
            self.seeking_ticks += 1
            if self.seeking_ticks > 30:
                self.centerer = 999
            if self.centerer >= self.CENTERING_TOLERANCE + 0.35: # Centering tolerance is the ultimate goal 
                                                                 # However hard to achieve in practice therefore we include buffer
                wx = self.search_center[0] + self.search_radius * math.cos(self.search_angle)
                wy = self.search_center[1] + self.search_radius * math.sin(self.search_angle)
                err = math.hypot(self.local_position.x - wx, self.local_position.y - wy)
                if err < 0.5:
                    self.search_angle += 0.8           # step around the circle
                    if self.search_angle > 2 * math.pi:
                        self.search_angle = 0.0
                        self.search_radius += 1.5       # expand outward each lap
                self.target_x = wx
                self.target_y = wy

        elif self.state == State.CENTERING:
            if not detection_fresh:
                # brief dropout: hold position and wait. only give up after ~2s.
                self.seeking_ticks += 1
                if self.seeking_ticks > 20:
                    self.state = State.SEEKING
                    self.seeking_ticks = 0
                    self.get_logger().info('Lost target (sustained), back to seeking')
                return
            self.seeking_ticks = 0   # fresh detection -> reset the dropout timer

            est_x = self.local_position.x + det_dx
            est_y = self.local_position.y + det_dy
            if self.centered_x is None:
                self.centered_x = est_x
                self.centered_y = est_y
            else:
                a = 0.2
                self.centered_x = (1 - a) * self.centered_x + a * est_x
                self.centered_y = (1 - a) * self.centered_y + a * est_y

            GAIN = 0.3
            MAX_STEP = 0.4
            dx = self.centered_x - self.local_position.x
            dy = self.centered_y - self.local_position.y
            dist = math.hypot(dx, dy)
            step_x = max(-MAX_STEP, min(MAX_STEP, dx * GAIN))
            step_y = max(-MAX_STEP, min(MAX_STEP, dy * GAIN))
            self.target_x = self.local_position.x + step_x
            self.target_y = self.local_position.y + step_y

            # Centered when the SMOOTHED estimate is close to the drone
            self.get_logger().info(
                f'CENTERING: det=({det_dx:.2f},{det_dy:.2f}) '
                f'est=({self.centered_x:.2f},{self.centered_y:.2f}) '
                f'drone=({self.local_position.x:.2f},{self.local_position.y:.2f}) '
                f'dist={dist:.2f}', throttle_duration_sec=0.5)
            
            
            if (dist < self.CENTERING_TOLERANCE+1.0 and dist > self.CENTERING_TOLERANCE+0.5):  # allow a bit more leniency to get to centering, then be strict
                self.get_logger().info('Within 2m of target, slowing down and moving down for precision')
                self.target_z = self.local_position.z + 0.3  # 0.5m below current, NED
                self.centerer = dist
            
            if (dist < self.CENTERING_TOLERANCE+0.5 and dist > self.CENTERING_TOLERANCE):  # once within 1.5m, slow down even more
                self.get_logger().info('Within 1.5m of target, slowing down even more for precision')
                self.target_z = self.local_position.z + 0.1  # 0.25m below current, NED
                self.centerer = dist

            if dist < self.CENTERING_TOLERANCE:
                self.state = State.DESCENDING
                self.get_logger().info(
                    f'Centered at ({self.centered_x:.2f}, {self.centered_y:.2f}), descending')
                self.centerer = dist
            if dist > self.CENTERING_TOLERANCE+1.0:
                self.centerer = 999

        elif self.state == State.DESCENDING:
            # keep re-tracking the cube while descending
            if detection_fresh:
                est_x = self.local_position.x + det_dx
                est_y = self.local_position.y + det_dy
                a = 0.2
                self.centered_x = (1 - a) * self.centered_x + a * est_x
                self.centered_y = (1 - a) * self.centered_y + a * est_y

            # same proportional approach as centering, so it homes in gently
            GAIN = 0.2
            MAX_STEP = 0.3
            dx = self.centered_x - self.local_position.x
            dy = self.centered_y - self.local_position.y
            step_x = max(-MAX_STEP, min(MAX_STEP, dx * GAIN))
            step_y = max(-MAX_STEP, min(MAX_STEP, dy * GAIN))
            self.target_x = self.local_position.x + step_x
            self.target_y = self.local_position.y + step_y

            self.target_z = self.local_position.z + 0.5

            if self.local_position.z > self.DESCEND_ALTITUDE_THRESHOLD:
                self.land()
                self.state = State.LANDING
                self.get_logger().info('Issuing NAV_LAND')

        elif self.state == State.LANDING:
            if self.vehicle_status.arming_state == 1:  # disarmed
                self.state = State.DONE
                self.get_logger().info('Mission complete')

    def print_status(self):
        with self.lock:
            self.get_logger().info(
                f'state={self.state.name}  color={self.target_color or "none"}  '
                f'pos=({self.local_position.x:.2f}, {self.local_position.y:.2f}, '
                f'{self.local_position.z:.2f})  det_age={self.detection_age_ticks}')

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()