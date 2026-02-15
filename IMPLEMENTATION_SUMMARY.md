# Implementation Summary: Workspace & Robot Base Frame Detection

## Overview

Extended the `weld_line_vision_pkg` to:
1. Define workspace using ArUco markers (ID 0,1,2,3)
2. Calculate robot base frame position using ArUco markers (ID 7, 17)
3. Visualize everything in RViz: workspace, trajectory, and full robot model

---

## Recent Meaningful Updates (Current Code)

This section records only updates that are currently present in code (reverted experiments excluded).

1. Camera-separated robot pose mode (PnP) is enabled by default.
   - Parameter: `use_camera_separate_calibration` (default `true`)
   - Code: `src/weld_line_vision_pkg/src/weld_line_vision_node.cpp`

2. Camera intrinsics are consumed from `/droidcam/camera_info` and validated before PnP.
   - Parameters: `camera_info_topic`, backward-compatible `info_topic`
   - Launch support: `camera_info_url`

3. Optional homography fallback for robot pose was added.
   - Parameter: `allow_homography_fallback_for_robot_pose` (default `false`)

4. Workspace and robot are both estimated in camera frame, then composed:
   - `T_ws_base = inv(T_cam_ws) * T_cam_base`
   - This is the active transform composition in `computeRobotBaseTransformFromCamera(...)`.

5. When image rotation is enabled (`rotate_image_90_cw=true`), marker corners are unrotated back for camera-PnP consistency.
   - Function: `unrotateCorners90CWToOriginal(...)`

6. Trajectory extraction is hard-gated by workspace mask.
   - Blue mask is ANDed with workspace polygon mask before skeletonization.
   - This prevents outside-blue regions from becoming trajectory candidates.

7. Workspace marker IDs are now launch-overrideable.
   - Launch args: `id_tl`, `id_tr`, `id_bl`, `id_br`
   - Useful for quick remapping without editing C++.

8. Robot base lateral translation offset is supported for fine alignment.
   - Parameter: `robot_base_y_offset_m`
   - Applied as translation offset only (orientation unchanged by this parameter).

9. ArUco-based intrinsics calibration utility was added.
   - Script: `src/weld_line_vision_pkg/scripts/aruco_intrinsics_calibrator.py`
   - Purpose: generate `droidcam.yaml` without checkerboard.
   - OpenCV ArUco API compatibility handled for multiple versions.

---

## Controller Target Interface (Current)

If you want to feed trajectory targets into a controller, this is the exact output format currently produced by `weld_line_vision`:

1. Main trajectory topic
   - Topic: `/weld_line_vision/weld_path`
   - Type: `nav_msgs/Path`
   - Frame: `workspace_frame` (from `frame_id` parameter)

2. Per-point value format (`Path.poses[i]`)
   - Type: `geometry_msgs/PoseStamped`
   - `pose.position.x`: workspace X (meters)
   - `pose.position.y`: workspace Y (meters)
   - `pose.position.z`: always `0.0` (workspace plane)
   - `pose.orientation`: identity (`w=1.0`, no orientation profile)

3. Important behavior
   - Points are sampled from detected skeleton in image order and transformed to workspace meters.
   - If valid points are insufficient, an empty/short path may be published and should be ignored by controller side.
   - No timing profile is included in `nav_msgs/Path` (controller/bridge must assign speed/time law).

4. For controller integration
   - First transform points from `workspace_frame` to the controller frame (typically `base_link`) via TF.
   - Then pass either:
     - Full waypoint list (`nav_msgs/Path`) to a trajectory planner, or
     - Single-step target (`geometry_msgs/PoseStamped` / numeric vector) in your control loop.
   - Keep tool orientation as a separate policy (because weld path currently contains position-only intent).

5. Existing reference in this repo
   - `src/trajectory_tracker/scripts/trajectory_publisher_node.py` republishes a path and emits one-point Cartesian target example:
   - `Float64MultiArray [x, y, z, enabled]`

---

## What Was Implemented

### 1. C++ Code Modifications (`weld_line_vision_node.cpp`)

**Added Features:**
- ✅ TF2 broadcaster for publishing transforms
- ✅ Detection of robot base markers (ID 7 = top, ID 17 = bottom, both 10cm)
- ✅ Computation of robot `base_link` frame from markers 7 and 17
- ✅ Workspace visualization marker (green rectangle boundary)
- ✅ Automatic TF publishing: `workspace_frame` → `base_link`

**Key Functions Added:**
```cpp
void publishWorkspaceMarker(const ros::Time& stamp)
// Publishes green rectangle showing workspace boundary

bool computeRobotBaseTransform(...)
// Detects markers 7 & 17, computes robot position and orientation
// Publishes TF transform
```

**Robot Base Calculation:**
- Position: Midpoint between markers 7 and 17
- Orientation: X-axis points from marker 17 → marker 7 (rotated -90°)
- Published as TF: `workspace_frame` → `base_link`

---

### 2. Launch File (`vision_with_robot_rviz.launch`)

**Launches:**
- USB camera node (DroidCam)
- Vision processing node
- Robot description (URDF)
- Robot state publisher
- Joint state publisher GUI
- RViz with custom configuration

**TF Tree:**
```
world (0, 0, 0.5m above workspace)
  └─ workspace_frame (defined by markers 0,1,2,3)
       ├─ weld_path
       └─ base_link (defined by markers 7,17)
            └─ robot links (shoulder, elbow, wrist, gripper, etc.)
```

---

### 3. RViz Configuration (`config/robot_workspace.rviz`)

**Displays:**
- Grid (world frame)
- TF frames (all transforms)
- Robot model (UR10 + Lacquey gripper)
- Workspace marker (green boundary)
- Weld path (red trajectory)
- Weld points (blue spheres)
- Debug images (ArUco detection + skeleton)

---

### 4. Dependencies Added

**CMakeLists.txt:**
```cmake
tf2_ros
tf2_geometry_msgs
```

**package.xml:**
```xml
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

---

## File Changes Summary

### Modified Files:
1. `src/weld_line_vision_pkg/src/weld_line_vision_node.cpp`
   - Added TF broadcasting
   - Added robot marker detection (ID 7, 17)
   - Added workspace visualization

2. `src/weld_line_vision_pkg/CMakeLists.txt`
   - Added tf2 dependencies

3. `src/weld_line_vision_pkg/package.xml`
   - Added tf2 dependencies

### New Files Created:
1. `src/weld_line_vision_pkg/launch/vision_with_robot_rviz.launch`
   - Complete system launch file

2. `src/weld_line_vision_pkg/config/robot_workspace.rviz`
   - RViz configuration

3. `src/weld_line_vision_pkg/README_USAGE.md`
   - User documentation

4. `src/weld_line_vision_pkg/scripts/generate_aruco_markers.py`
   - Marker generation utility

---

## How to Use

### Quick Start:
```bash
# 1. Build the workspace
cd ~/welding_robot
source /opt/ros/noetic/setup.bash
catkin build weld_line_vision
source devel/setup.bash

# 2. Launch the system
roslaunch weld_line_vision vision_with_robot_rviz.launch

# 3. Generate ArUco markers (optional)
roscd weld_line_vision
cd scripts
python3 generate_aruco_markers.py
```

### Marker Setup:
1. Print markers from `aruco_markers/` directory
2. Workspace markers (6cm): Place at corners of A4 paper
   - ID 0: Top-left
   - ID 1: Top-right
   - ID 2: Bottom-left
   - ID 3: Bottom-right
3. Robot markers (10cm): Attach to robot base
   - ID 7: Front/top (robot forward direction)
   - ID 17: Back/bottom

---

## Technical Details

### Homography Transform
- **Input**: 4 workspace marker centers (pixels)
- **Output**: Homography matrix `H_img_to_paper`
- **Purpose**: Convert pixel coordinates → workspace coordinates (meters)

### Robot Base Transform
- **Input**: Markers 7 & 17 centers (pixels)
- **Process**:
  1. Transform to workspace coordinates using homography
  2. Calculate midpoint (robot position)
  3. Calculate orientation from vector 17→7
- **Output**: TF transform `workspace_frame` → `base_link`

### Coordinate Systems
- **world**: Fixed reference (0.5m above workspace)
- **workspace_frame**: Origin at workspace corner (marker-based)
- **base_link**: Robot base (calculated from markers 7, 17)
- **camera_optical_frame**: Camera perspective

---

## ROS Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/weld_line_vision/weld_path` | nav_msgs/Path | Welding trajectory |
| `/weld_line_vision/weld_points_marker` | visualization_msgs/Marker | Point cloud visualization |
| `/weld_line_vision/workspace_marker` | visualization_msgs/Marker | Workspace boundary |
| `/weld_line_vision/debug_image` | sensor_msgs/Image | ArUco + skeleton |
| `/weld_line_vision/debug_mask` | sensor_msgs/Image | Color segmentation |
| `/weld_line_vision/debug_skeleton` | sensor_msgs/Image | Skeleton image |
| `/tf` | tf2_msgs/TFMessage | All transforms |

---

## Parameters

### ArUco Detection:
```xml
<param name="aruco_dict_id" value="10"/>  <!-- DICT_4X4_50 -->
<param name="aruco_marker_size" value="0.06"/>  <!-- 6cm workspace -->
<param name="robot_marker_size" value="0.10"/>  <!-- 10cm robot -->
```

### Marker IDs:
```xml
<!-- Workspace corners (6cm) -->
<param name="id_tl" value="0"/>
<param name="id_tr" value="1"/>
<param name="id_bl" value="2"/>
<param name="id_br" value="3"/>

<!-- Robot base (10cm) -->
<param name="id_robot_top" value="7"/>
<param name="id_robot_bottom" value="17"/>
```

### Color Detection (Cyan/Blue):
```xml
<param name="h_low" value="75"/>
<param name="h_high" value="130"/>
<param name="s_low" value="20"/>
<param name="v_low" value="40"/>
```

---

## Verification

### Check TF Tree:
```bash
rosrun rqt_tf_tree rqt_tf_tree
# Should show: world → workspace_frame → base_link → robot joints
```

### Check Topics:
```bash
rostopic list | grep weld_line_vision
rostopic echo /weld_line_vision/weld_path
```

### View Images:
```bash
rosrun image_view image_view image:=/weld_line_vision/debug_image
```

---

## Known Limitations

1. Robot markers (7, 17) must both be visible for robot TF to be published
2. Workspace markers (0,1,2,3) must all be visible for homography
3. Weld line detection requires proper lighting and color tuning
4. Robot appears at origin if markers not detected

---

## Next Steps / Improvements

1. Add pose estimation for 3D orientation (currently 2D plane)
2. Implement marker tracking/filtering for smoother TF
3. Add configuration GUI for HSV tuning
4. Support partial marker detection (3/4 markers)
5. Add workspace calibration wizard

---

## Build Status

✅ **Successfully compiled** on ROS Noetic
✅ **All dependencies** installed (tf2_ros, OpenCV, ArUco)
✅ **Launch files** tested
✅ **RViz config** created

---

## References

- ArUco Dictionary: DICT_4X4_50 (cv::aruco::DICT_4X4_50 = 10)
- Robot Model: `tum_ics_ur10_robot/tum_ics_ur10_description/urdf/ur10_robot_lacquey.urdf.xacro`
- Coordinate conventions: ROS REP-103 (right-handed, Z-up)

---

**Author**: Claude Code Assistant
**Date**: 2026-02-14
**Package**: weld_line_vision
**Branch**: detection
