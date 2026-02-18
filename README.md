# Welding Robot

Autonomous weld-line tracking system for a **UR10** robot arm. A camera detects the weld seam via ArUco markers and colour-based vision, and an operational-space controller follows the extracted trajectory while an admittance filter regulates contact force through a Schunk NetBox F/T sensor.

## Overview

| Component | Description |
|---|---|
| **weld_line_vision_pkg** | Detects workspace & weld line from a camera (DroidCam / USB), publishes `nav_msgs/Path` |
| **cpp_controller** | Operational-space controller (PD + admittance) that tracks the weld path in Cartesian space |
| **tum_ics_ur10_robot** | UR10 robot driver and bringup |
| **tum_ics_schunk_netbox** | Schunk NetBox force/torque sensor driver |
| **ur_model_cpp** | UR10 kinematics & dynamics model |
| **utilities** | Shared helper libraries |

## Prerequisites

- **ROS Noetic** on Ubuntu 20.04
- [DroidCam](https://www.dev47apps.com/) installed on both iPhone and PC (used as the vision camera)
- Schunk NetBox F/T sensor connected via Ethernet
- UR10 robot reachable on the network

## Building

```bash
cd ~/welding_robot
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## Running the System

Open **six terminals**. In each one, first source the workspace:

```bash
source ~/welding_robot/devel/setup.bash
```

### Terminal 1 — ROS Core

```bash
rosclean purge -y   # optional: clear old logs
roscore
```

### Terminal 2 — UR10 Robot Driver

```bash
roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch
```

### Terminal 3 — Force/Torque Sensor

```bash
roslaunch tum_ics_schunk_netbox sensor_publisher.launch
```

### Terminal 4 — DroidCam (iPhone Camera)

Connect the iPhone via USB, open DroidCam on the phone, then:

```bash
droidcam-cli ios 4747 -v
```

### Terminal 5 — Weld Line Vision + RViz

```bash
roslaunch weld_line_vision vision_with_robot_rviz.launch
```

This launches the vision pipeline (ArUco detection, weld-line extraction) and opens RViz for visualisation.

### Terminal 6 — Controller

```bash
roslaunch cpp_controller operational_space_controller_6dof.launch
```

The controller moves through four phases:

1. **INIT** — Joint-space interpolation to a non-singular starting pose
2. **LINEAR** — Cartesian-space linear motion to the first weld-path waypoint
3. **TRACK_SLOW** — Follows the detected weld path with admittance-based force control
4. **TRACK_FAST** — Follows the detected weld path with admittance-based force control
5. **END** — Holds the final joint configuration

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/weld_line_vision/weld_path` | `nav_msgs/Path` | Detected weld trajectory (workspace frame) |
| `/schunk_netbox/raw` | `geometry_msgs/WrenchStamped` | Raw force/torque readings |
| `/desired_trajectory_path` | `nav_msgs/Path` | Controller's desired trajectory (RViz) |
| `/admittance_state` | `Float32MultiArray` | Admittance offset & external force (debug) |

## Further Documentation

- [QUICK_START.md](QUICK_START.md) — Marker setup and vision quick-start guide
- [VISUAL_CHECK_GUIDE.md](VISUAL_CHECK_GUIDE.md) — How to verify the vision pipeline
- [force_torque_sensor_setup_guide.md](force_torque_sensor_setup_guide.md) — F/T sensor configuration
- [gripper_setup_guide.md](gripper_setup_guide.md) — Gripper setup instructions
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Detailed implementation notes