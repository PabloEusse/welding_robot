# Quick Start Guide - Weld Line Vision with Robot

## 1. Build (First Time Only)
```bash
cd ~/welding_robot
source /opt/ros/noetic/setup.bash
catkin build weld_line_vision
source devel/setup.bash
```

## 2. Generate Markers (Optional)
```bash
roscd weld_line_vision
cd scripts
python3 generate_aruco_markers.py
# Markers saved in aruco_markers/ directory
# Print at 300 DPI, actual size
```

## 3. Setup Markers

### Workspace Markers (6cm each):
```
   [ID 0]----------[ID 1]
      |              |
      |  Workspace   |
      |  (A4 paper)  |
      |              |
   [ID 2]----------[ID 3]
```

### Robot Markers (10cm each):
```
Robot Base:
   [ID 7]  ← Front/Top
     |
     | (same plane)
     |
   [ID 17] ← Back/Bottom
```

## 4. Launch System
```bash
source ~/welding_robot/devel/setup.bash
roslaunch weld_line_vision vision_with_robot_rviz.launch
```

## 5. What You'll See

**RViz Windows:**
- Main 3D view: Robot + workspace + trajectory
- Debug image: Camera view with markers
- Joint sliders: Move robot manually

**Terminal Output:**
```
[ INFO] ArUco ids: TL=0 TR=1 BL=2 BR=3
[ INFO] Robot base: (0.105, 0.148, yaw=45.0 deg)
```

## 6. Troubleshooting

### No markers detected:
```bash
# Check camera
rostopic hz /droidcam/image_raw

# View debug image
rosrun image_view image_view image:=/weld_line_vision/debug_image
```

### Robot not showing:
```bash
# Check TF
rosrun rqt_tf_tree rqt_tf_tree

# Look for: world → workspace_frame → base_link
```

### No weld line:
```bash
# View color mask
rosrun image_view image_view image:=/weld_line_vision/debug_mask

# Adjust HSV in launch file if needed
```

## 7. Adjust Parameters

Edit `vision_with_robot_rviz.launch`:

**Camera:**
```xml
<arg name="video_device" default="/dev/video0" />
<arg name="image_width" default="640" />
```

**Colors (Cyan/Blue):**
```xml
<param name="h_low" value="75"/>    <!-- Hue min -->
<param name="h_high" value="130"/>  <!-- Hue max -->
<param name="s_low" value="20"/>    <!-- Saturation min -->
<param name="v_low" value="40"/>    <!-- Value min -->
```

**Marker IDs:**
```xml
<param name="id_robot_top" value="7"/>     <!-- Change if needed -->
<param name="id_robot_bottom" value="17"/>
```

## 8. Useful Commands

```bash
# List topics
rostopic list | grep weld

# Echo weld path
rostopic echo /weld_line_vision/weld_path

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Check node info
rosnode info /weld_line_vision

# Record data
rosbag record /weld_line_vision/weld_path /tf
```

## 9. Files Created

```
weld_line_vision_pkg/
├── src/
│   └── weld_line_vision_node.cpp         (modified)
├── launch/
│   └── vision_with_robot_rviz.launch     (new)
├── config/
│   └── robot_workspace.rviz              (new)
├── scripts/
│   └── generate_aruco_markers.py         (new)
├── CMakeLists.txt                        (modified)
├── package.xml                           (modified)
└── README_USAGE.md                       (new)
```

## 10. Expected Result

When all markers are detected:
- ✅ Green workspace rectangle visible in RViz
- ✅ Robot model positioned at markers 7 & 17
- ✅ Red weld path following the detected line
- ✅ Blue points along the trajectory
- ✅ All TF frames showing in TF tree

---

**Need Help?**
- See `README_USAGE.md` for detailed documentation
- See `IMPLEMENTATION_SUMMARY.md` for technical details
- Check ROS logs: `~/.ros/log/latest/`
