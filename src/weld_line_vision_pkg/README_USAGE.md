# Weld Line Vision with Robot Visualization

This package uses ArUco markers to define the workspace and robot base frame, detects weld lines, and visualizes everything in RViz.

## System Configuration

### ArUco Marker Setup

**Workspace Markers (6cm):**
- ID 0: Top Left
- ID 1: Top Right
- ID 2: Bottom Left
- ID 3: Bottom Right

**Robot Base Markers (10cm):**
- ID 7: Top marker (robot front direction)
- ID 17: Bottom marker (robot back direction)

### Coordinate Frame Structure

```
world
  └─ base_link (robot base, defined by markers ID 7, 17)
       ├─ workspace_frame (defined by markers ID 0,1,2,3)
       │    └─ weld_path (welding trajectory)
       └─ robot (UR10 + gripper)
```

**Note:** The robot's base_link is now the primary reference frame. The workspace frame and weld trajectories are expressed relative to the robot base, making it easier to command the robot to follow trajectories.

## Usage

### 1. Setup Environment
```bash
cd ~/welding_robot
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 2. Launch the System
```bash
roslaunch weld_line_vision vision_with_robot_rviz.launch
```

This launches:
- USB camera node (DroidCam)
- Vision processing node (workspace + robot base detection)
- Robot model (UR10 + Lacquey gripper)
- RViz visualization

### 3. What You'll See in RViz

- **TF Frames**: All coordinate frames (world, workspace_frame, base_link, robot links)
- **Robot Model**: Full UR10 robot + gripper
- **Workspace Marker**: Green rectangle (workspace boundary)
- **Weld Path**: Red path (detected weld line)
- **Weld Points**: Blue spheres (sampled weld points)
- **Debug Image**: ArUco marker detection + skeleton overlay

### 4. Joint State Publisher GUI

Use the Joint State Publisher GUI window to manually move robot joints.

## Parameter Tuning

Key parameters you can modify in the launch file:

### Camera Settings
```xml
<arg name="video_device" default="/dev/video0" />
<arg name="image_width" default="640" />
<arg name="image_height" default="480" />
```

### Workspace Size
```xml
<param name="paper_width_m" value="0.210"/>   <!-- A4 width -->
<param name="paper_height_m" value="0.297"/>  <!-- A4 height -->
```

### ArUco Marker IDs
```xml
<!-- Workspace markers -->
<param name="id_tl" value="0"/>
<param name="id_tr" value="1"/>
<param name="id_bl" value="2"/>
<param name="id_br" value="3"/>

<!-- Robot base markers -->
<param name="id_robot_top" value="7"/>
<param name="id_robot_bottom" value="17"/>
```

### Color Detection (HSV)
```xml
<param name="h_low" value="75"/>
<param name="h_high" value="130"/>
<param name="s_low" value="20"/>
<param name="v_low" value="40"/>
```

## Troubleshooting

### Markers Not Detected
1. Check lighting conditions
2. Adjust camera focus
3. Verify marker sizes (workspace: 6cm, robot: 10cm)
4. Confirm correct marker IDs

### Robot Not Appearing in RViz
1. Check TF tree: `rosrun rqt_tf_tree rqt_tf_tree`
2. Verify robot markers (ID 7, 17) are detected
3. Look for "Robot base: ..." messages in terminal output

### Weld Line Not Detected
1. Adjust HSV values (cyan/blue range)
2. Improve lighting conditions
3. Check `/weld_line_vision/debug_mask` topic

## ROS Topics

Published topics:
- `/weld_line_vision/weld_path` - Weld trajectory (nav_msgs/Path)
- `/weld_line_vision/weld_points_marker` - Weld points visualization
- `/weld_line_vision/workspace_marker` - Workspace boundary
- `/weld_line_vision/debug_image` - Debug image with markers
- `/tf` - TF transforms (workspace_frame, base_link)

## Generating ArUco Markers

To generate ArUco markers:
```python
import cv2
import cv2.aruco as aruco

# Use DICT_4X4_50 dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Generate markers (ID, size in pixels)
for marker_id in [0, 1, 2, 3, 7, 17]:
    img = aruco.drawMarker(aruco_dict, marker_id, 200)  # 200x200 pixels
    cv2.imwrite(f'marker_{marker_id}.png', img)
```

Print at actual size:
- ID 0,1,2,3: 6cm x 6cm
- ID 7, 17: 10cm x 10cm

## Notes

- Robot base frame (base_link) is defined by markers ID 7 (front) and ID 17 (back)
- Robot X-axis points from marker ID 17 → ID 7
- Workspace frame is defined by the center points of markers ID 0,1,2,3
- **All coordinates (including weld trajectories) are relative to base_link (robot base frame)**
- This allows direct trajectory commands in robot coordinates
