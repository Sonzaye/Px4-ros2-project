"""
PX4 Keyboard Teleoperation using Tkinter

A Tkinter window captures keypresses while focused, sending offboard
commands to PX4.

Key bindings:
  T          takeoff (arm + offboard + climb to 5m) (offboard means goes into offboard mode)
  L          land
  W/S        move north/south
  A/D        move east/west
  UP/DOWN    altitude up/down
  Q/E        yaw left/right
  SPACE      arm/disarm toggle
  ESC        quit

PX4 uses NED: +X north, +Y east, +Z down (z=-5 means 5m up).
"""

import math
import threading
import tkinter as tk

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


class KeyboardController(Node):
    MAX_LEASH_XY = 1.5    # max meters target can lead current position
    MAX_LEASH_Z = 0.75    # max meters target can lead vertically (takes a while for drone inputs to be registered)
    MOVE_STEP = 0.25      
    ALT_STEP = 0.25       
    YAW_STEP = math.radians(10)  #value can be changed later, somewhat arbitrary
    TAKEOFF_ALTITUDE = -5.0
    NAV_STATE_AUTO_LAND = 18

    def __init__(self, gui):
        super().__init__('keyboard_controller')
        self.gui = gui

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_cb, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_cb, qos)

        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self.lock = threading.Lock()

        self.create_timer(0.1, self.loop)
        self.create_timer(0.5, self.update_gui)

    def handle_key(self, event):
        keysym = event.keysym.lower()
        with self.lock:
            if keysym == 't':
                self.engage_offboard_mode()
                self.force_arm()
                self.target_z = self.TAKEOFF_ALTITUDE
                self.gui.flash_status('Takeoff')
            elif keysym == 'l':
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.gui.flash_status('Land')
            elif keysym == 'w':
                self.target_x += self.MOVE_STEP
            elif keysym == 's':
                self.target_x -= self.MOVE_STEP
            elif keysym == 'd':
                self.target_y += self.MOVE_STEP
            elif keysym == 'a':
                self.target_y -= self.MOVE_STEP
            elif keysym == 'q':
                self.target_yaw -= self.YAW_STEP
            elif keysym == 'e':
                self.target_yaw += self.YAW_STEP
            elif keysym == 'up':
                self.target_z -= self.ALT_STEP
            elif keysym == 'down':
                self.target_z += self.ALT_STEP
            # Leash the target to the drone's current position so spamming
            # keys doesn't let the target run away.
            dx = self.target_x - self.local_position.x
            dy = self.target_y - self.local_position.y
            dist_xy = math.sqrt(dx*dx + dy*dy)
            if dist_xy > self.MAX_LEASH_XY:
                scale = self.MAX_LEASH_XY / dist_xy
                self.target_x = self.local_position.x + dx * scale
                self.target_y = self.local_position.y + dy * scale

            dz = self.target_z - self.local_position.z
            if abs(dz) > self.MAX_LEASH_Z:
                self.target_z = self.local_position.z + math.copysign(self.MAX_LEASH_Z, dz)
            elif keysym == 'space':
                if self.vehicle_status.arming_state == 2:
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        param1=0.0, param2=21196.0)
                    self.gui.flash_status('Disarm')
                else:
                    self.force_arm()
                    self.gui.flash_status('Arm')
            elif keysym == 'escape':
                self.gui.quit()

    def update_gui(self):
        # .after is used for thread-safety measures
        with self.lock:
            tx, ty, tz, tyaw = self.target_x, self.target_y, self.target_z, self.target_yaw

        pos = (self.local_position.x, self.local_position.y, self.local_position.z)
        target = (tx, ty, tz, tyaw)
        arm = self.vehicle_status.arming_state
        nav = self.vehicle_status.nav_state

        self.gui.root.after(0, lambda: self.gui.update(
            pos=pos, target=target, arm=arm, nav=nav
        ))

    # --- PX4 commands ---
    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def force_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0, param2=21196.0)

    # --- ROS callbacks ---
    def local_position_cb(self, msg):
        self.local_position = msg

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    # --- Publishers ---
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

    def loop(self):
        if self.vehicle_status.nav_state != self.NAV_STATE_AUTO_LAND:
            self.publish_offboard_heartbeat()
            self.publish_position_setpoint()

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


class ControllerGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Drone Controller')
        self.root.geometry('400x300')
        self.root.configure(bg='#1e1e1e')

        font = ('Monospace', 11)
        fg = '#e0e0e0'
        bg = '#1e1e1e'

        self.help_label = tk.Label(
            self.root,
            text=('T takeoff   L land\n'
                  'W/S north/south   A/D east/west\n'
                  '↑/↓ altitude   Q/E yaw\n'
                  'SPACE arm toggle   ESC quit'),
            font=font, fg=fg, bg=bg, justify='left'
        )
        self.help_label.pack(pady=10)

        self.status_label = tk.Label(
            self.root, text='Waiting for PX4...',
            font=('Monospace', 10), fg='#aaaaaa', bg=bg, justify='left'
        )
        self.status_label.pack(pady=10)

        self.flash_label = tk.Label(
            self.root, text='', font=('Monospace', 14, 'bold'),
            fg='#00ff88', bg=bg
        )
        self.flash_label.pack(pady=5)

        self.node = None  # set later

    def attach_node(self, node):
        self.node = node
        self.root.bind('<KeyPress>', node.handle_key)
        self.root.focus_set()

    def update(self, pos, target, arm, nav):
        text = (
            f'pos    ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n'
            f'target ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}, '
            f'{math.degrees(target[3]):.0f}°)\n'
            f'arm={arm}   nav={nav}'
        )
        self.status_label.config(text=text)

    def flash_status(self, message):
        def do_flash():
            self.flash_label.config(text=message)
            self.root.after(1500, lambda: self.flash_label.config(text=''))
        self.root.after(0, do_flash)

    def quit(self):
        self.root.quit()

    def run(self):
        self.root.mainloop()


def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    gui = ControllerGUI()
    node = KeyboardController(gui)
    gui.attach_node(node)

    # Spin ROS in a background thread; Tkinter owns the main thread
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    try:
        gui.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()