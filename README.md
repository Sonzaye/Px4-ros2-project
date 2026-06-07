# PX4 ROS 2 Autonomous Drone

Autonomous quadcopter simulation: PX4 SITL + ROS 2 + Gazebo, with custom
offboard control, waypoint missions, and YOLOv8 perception on the camera feed.
The goal is to progressively advance the default drone, with custom waypoint planners, new add-ons etc.


## What it does

- Arms, takes off, and lands autonomously through a custom ROS 2 node
- Flies multi-waypoint missions with a state-machine planner ![Waypoint_demo](docs/demo.gif)
- Streams the drone's camera and runs YOLOv8 object detection on every frame ![Camera_demo](docs/camera_demo.gif)
- Visualizes pose, waypoints, and trajectory live in RViz


Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · PX4 SITL (main) · uXRCE-DDS · YOLOv8


## Architecture

![Architecture](docs/architecture.png)

## Nodes

| Node | Role |
|---|---|
| `takeoff_and_hover` | Arm, takeoff, hover, land |
| `waypoint_mission` | State-machine waypoint navigation |
| `mission_visualizer` | RViz TF, markers, and trajectory |
| `yolo_detector` | YOLOv8 inference on the camera feed |

## Roadmap

- [x] PX4 ↔ ROS 2 bridge via uXRCE-DDS
- [x] Offboard control, waypoint missions, RViz visualization
- [x] YOLO perception pipeline
- [ ] Closed-loop perception (hover over detected target)
- [ ] Real hardware deployment (Pixhawk)
```
