# PX4 ROS 2 Autonomous Drone

An autonomous drone system built on PX4 SITL, ROS 2 Lyrical, and Gazebo Jetty.

## Stack
- **OS:** Ubuntu 26.04 Resolute Raccoon
- **ROS 2:** Lyrical Luth
- **Simulation:** Gazebo Jetty (gz-sim 10.1.1)
- **Flight Controller:** PX4 SITL (v1.18.0)
- **Ground Control:** QGroundControl v5.0.8

## Project Structure
- `px4/` — PX4 SITL configuration and launch
- `ros2_ws/` — ROS 2 workspace (mission planner, perception, control nodes)
- `docs/` — Architecture diagrams and notes

## Roadmap
- [x] PX4 SITL + Gazebo Jetty simulation running
- [x] QGroundControl connected via MAVLink
- [ ] MAVROS bridge — ROS 2 ↔ PX4 communication
- [ ] Offboard control node — autonomous takeoff/land
- [ ] Waypoint mission planner
- [ ] YOLO perception node
- [ ] Real hardware deployment (Pixhawk)

## Setup
```bash
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Launch simulation
cd PX4-Autopilot
make px4_sitl gz_x500
```
