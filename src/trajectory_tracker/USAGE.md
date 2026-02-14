# Trajectory Tracker - Usage Guide

Vision-based trajectory tracking system for UR10 welding robot.

## Quick Demo (One Command)

**Launch everything (Robot + Tracker + RViz):**

```bash
roslaunch trajectory_tracker demo.launch
```

This will:
- ✓ Load UR10 robot model
- ✓ Start camera and marker detection
- ✓ Open RViz with all visualizations
- ✓ Show camera debug window
- ✓ Print instructions in terminal

Then follow the on-screen instructions to calibrate and extract trajectory.

---

## Detailed Setup

### 1. Print ArUco Markers

Generate and print 4 markers (10cm x 10cm each):

```bash
cd ~/TUM/2025w/MSBRDM/welding_robot/src/trajectory_tracker/scripts
python3 generate_aruco_markers.py
```

Or use online generator: https://chev.me/arucogen/
- Dictionary: **4x4 (50, 100, 250, 1000)**
- Marker IDs: **3, 7, 17, 11**
- Size: **100mm** (10cm)

**Print at exact size - verify with ruler!**

### 2. Place Markers

Arrange markers at workspace corners (looking from above):

```
    ID 3 (TL) ----------- ID 7 (TR)
       |                      |
       |     WORKSPACE        |
       |                      |
   ID 11 (BL) ---------- ID 17 (BR)
```

**Important**:
- Keep markers FLAT and PARALLEL to surface
- Place OUTSIDE drawing area (at corners)
- All 4 must be visible to camera
- Avoid shadows/reflections

### 3. Launch System

```bash
roslaunch trajectory_tracker trajectory_tracker.launch show_gui:=true
```

### 4. Calibrate Workspace

```bash
rosservice call /trajectory_tracker/calibrate_workspace "{marker_ids: [3, 7, 17, 11]}"
```

Expected output:
```
WORKSPACE CALIBRATED
  Top-Left     (ID  3): ( x.xxx,  y.yyy,  z.zzz)
  Top-Right    (ID  7): ( x.xxx,  y.yyy,  z.zzz)
  Bottom-Right (ID 17): ( x.xxx,  y.yyy,  z.zzz)
  Bottom-Left  (ID 11): ( x.xxx,  y.yyy,  z.zzz)
  Workspace size: X.XXXm x Y.YYYm
```

### 5. Draw Line

Draw a line on paper within the workspace using a **blue pen/marker**.

### 6. Extract Line

```bash
rosservice call /trajectory_tracker/extract_line
```

---

## Visualization

### RViz Setup

1. Launch RViz:
```bash
rosrun rviz rviz
```

2. Configure:
   - **Fixed Frame**: `base_link`
   - Add displays:
     - **MarkerArray** → `/trajectory_tracker/line_contour_markers` (red boundary)
     - **MarkerArray** → `/trajectory_tracker/line_skeleton_markers` (green centerline)
     - **Path** → `/trajectory_tracker/extracted_path`

3. Color coding:
   - 🔴 **Red**: Line boundary (contour)
   - 🟢 **Green**: Extracted trajectory (skeleton)
   - 🟡 **Yellow sphere**: Start point
   - 🟣 **Magenta sphere**: End point

### Debug Images

View detection in real-time:
```bash
rosrun rqt_image_view rqt_image_view
```

Select topics:
- `/marker_detector/debug_image` - Marker detection
- `/trajectory_tracker/extraction_debug` - Line extraction

---

## Configuration

### Camera Position

Edit `config/tracker_config.yaml`:

```yaml
coordinate_transformer_node:
  # Camera pose in robot base frame [x, y, z, roll, pitch, yaw]
  camera_to_robot: [0.5, 0.0, 0.8, 3.14159, 0.0, 0.0]

  # Drawing surface Z height in robot frame
  plane_z: 0.0
```

### Line Color

To detect different colored lines, edit `config/tracker_config.yaml`:

```yaml
line_extractor_node:
  # For BLUE line (default):
  color_lower1: [100, 100, 100]
  color_upper1: [130, 255, 255]

  # For RED line:
  # color_lower1: [0, 100, 100]
  # color_upper1: [10, 255, 255]
  # color_lower2: [160, 100, 100]
  # color_upper2: [180, 255, 255]

  # For GREEN line:
  # color_lower1: [40, 100, 100]
  # color_upper1: [80, 255, 255]
```

### Smoothing Settings

Adjust trajectory smoothing in `config/tracker_config.yaml`:

```yaml
line_extractor_node:
  enable_smoothing: true
  smoothing_method: "bspline"  # Options: bspline, savgol, gaussian

  # B-spline smoothing (0-100, higher = smoother)
  bspline_smoothing: 5.0

  # Savitzky-Golay filter (odd numbers)
  savgol_window: 11
  savgol_order: 3

  # Gaussian kernel size (odd numbers)
  gaussian_kernel: 5
```

**Smoothing comparison**:
- **B-spline** (recommended): Smoothest curves, best for curved lines
- **Savgol**: Preserves features while smoothing
- **Gaussian**: Simple averaging, faster but less smooth

---

## Troubleshooting

### Markers not detected
- ✓ Check `/marker_detector/debug_image` topic
- ✓ Ensure markers are flat and well-lit
- ✓ Verify markers are in camera view
- ✓ Check marker size in config matches printed size

### Line extraction fails
- ✓ Verify line color matches config
- ✓ Ensure line is within calibrated workspace
- ✓ Check line is visible in `/camera/image_raw`
- ✓ Try adjusting HSV color ranges

### Inaccurate coordinates
- ✓ Recalibrate workspace
- ✓ Verify camera position in config
- ✓ Check marker placement is accurate
- ✓ Ensure plane_z height is correct

### Path too jagged
```yaml
# Increase smoothing
bspline_smoothing: 20.0  # Default: 5.0

# Or increase sampling distance
sampling_distance: 10  # Default: 5
```

### Path too smooth (detail loss)
```yaml
# Decrease smoothing
bspline_smoothing: 1.0

# Or use Savitzky-Golay filter
smoothing_method: "savgol"
savgol_window: 7
```

---

## Advanced Usage

### Custom Marker IDs

Use any 4 different marker IDs:

```bash
# Example: Use markers 5, 10, 15, 20
rosservice call /trajectory_tracker/calibrate_workspace "{marker_ids: [5, 10, 15, 20]}"
```

Remember: Order is [Top-Left, Top-Right, Bottom-Right, Bottom-Left]

### HSV Color Tuning

Find optimal HSV ranges with live tuning:

```bash
rosrun rqt_image_view rqt_image_view
```

Select `/camera/image_raw` and use color threshold plugin.

---

## System Architecture

```
Camera → Workspace Detection → Line Extraction
   ↓          (Marker + Transform)         ↓
Image      ArUco Markers + Calib     Trajectory Path
```

**Key Nodes**:
- `camera_node` - Image acquisition
- `workspace_detector` - ArUco marker detection + coordinate transformation + workspace calibration
- `line_extractor_node` - Line detection and skeleton extraction
- `trajectory_publisher_node` - Trajectory republishing

**Key Topics**:
- `/camera/image_raw` - Raw camera feed
- `/marker_detector/markers` - Detected markers
- `/trajectory_tracker/extracted_path` - Final trajectory
- `/trajectory_tracker/line_contour_markers` - Line boundary visualization
- `/trajectory_tracker/line_skeleton_markers` - Centerline visualization

---

## Support

For issues or questions:
- Check `/rosout` logs for error messages
- Verify all nodes are running: `rosnode list`
- Test camera: `rosrun rqt_image_view rqt_image_view`

---

## Lab Testing Guide (Complete Workflow)

### Before You Go to Lab

**Required Items:**
1. ✅ Laptop with this code
2. ✅ DroidCam app installed on phone
3. ✅ USB cable (for phone connection)
4. ✅ 4 printed ArUco markers (IDs: 3, 7, 17, 11) - **10cm x 10cm size**
5. ✅ Blue pen/marker for drawing
6. ✅ White paper for workspace
7. ✅ Tape to secure markers

**Pre-check:**
```bash
# Check all files are executable
cd ~/TUM/2025w/MSBRDM/welding_robot/src/trajectory_tracker/scripts
ls -l *.py | grep rwx

# If any file is not executable:
chmod +x *.py
```

---

### Step-by-Step Lab Procedure

#### **STEP 1: Setup Camera Connection**

**Terminal 1: Connect DroidCam**
```bash
# Connect phone via USB
# On phone: Open DroidCam app
# On laptop:
droidcam-cli -a

# You should see: "Client connected" message
# Camera will appear as /dev/video0
```

**Verify camera works:**
```bash
# Test camera with v4l2
v4l2-ctl --list-devices

# Should show: DroidCam or similar device at /dev/video0
```

---

#### **STEP 2: Place ArUco Markers**

**Physical Setup:**
1. Place white paper on table (this is your workspace)
2. Place 4 markers at corners **OUTSIDE** the paper (markers define workspace boundary)
3. **CRITICAL**: Markers must be FLAT and PARALLEL to table surface
4. Use tape to secure them firmly

**Marker Layout (looking from above camera):**
```
     ID 3 (TL)  ----------  ID 7 (TR)
        |                        |
        |    White Paper         |
        |    (Workspace)         |
        |                        |
    ID 11 (BL)  ----------  ID 17 (BR)
```

**Tips:**
- Keep all 4 markers visible to camera
- Ensure good lighting (no shadows on markers)
- Markers should be at same height (flat on table)
- Distance between markers ~30-50cm

---

#### **STEP 3: Launch System**

**Terminal 2: Start ROS System**
```bash
# Navigate to workspace
cd ~/TUM/2025w/MSBRDM/welding_robot

# Source ROS
source devel/setup.bash

# Launch demo (this starts EVERYTHING)
roslaunch trajectory_tracker demo.launch

# You should see:
# - Camera preview window opens
# - RViz opens with robot model
# - 5 nodes start successfully
# - Instructions printed in terminal
```

**What you should see:**
- **Camera Window**: Live camera feed with marker detection overlay
- **RViz Window**: UR10 robot model in 3D view
- **Terminal**: Green "[INFO]" messages (not red "[ERROR]")

**If camera window doesn't show:**
- Check DroidCam is still connected
- Try: `ls /dev/video*` to verify camera device

**If RViz doesn't show robot:**
- Click "Add" → "RobotModel" in RViz
- Set Fixed Frame to "base_link"

---

#### **STEP 4: Verify Marker Detection**

**Check camera window:**
- You should see **GREEN BOXES** around detected markers
- Each marker shows **"ID: X"** label
- **Coordinate axes** (RGB lines) on each marker

**If markers not detected:**
```bash
# Terminal 3: Check debug image
rosrun rqt_image_view rqt_image_view

# Select topic: /marker_detector/debug_image
# You should see markers with green boxes and ID labels
```

**If still no detection:**
- Check marker print quality (sharp black/white contrast)
- Verify markers are 10cm x 10cm (measure with ruler!)
- Improve lighting
- Move camera closer

**Check in RViz:**
- You should see **BLUE SPHERES** at marker positions in 3D space
- Each sphere has a text label "ID: X"
- If you see spheres → marker detection + transformation working! ✅

---

#### **STEP 5: Calibrate Workspace**

**Terminal 3: Run Calibration Service**
```bash
# Source ROS first
source ~/TUM/2025w/MSBRDM/welding_robot/devel/setup.bash

# Call calibration service with marker IDs [TL, TR, BR, BL]
rosservice call /trajectory_tracker/calibrate_workspace "{marker_ids: [3, 7, 17, 11]}"
```

**Expected Output:**
```
success: True
message: "Workspace calibrated! Size: 0.XXXm x 0.YYYm"
workspace:
  is_calibrated: True
  marker_ids: [3, 7, 17, 11]
  corners: [...]
  width: 0.XXX
  height: 0.YYY
```

**Also check Terminal 2 (where demo.launch is running):**
```
============================================================
WORKSPACE CALIBRATED
============================================================
  Top-Left     (ID  3): ( 0.XXX,  0.YYY,  0.ZZZ)
  Top-Right    (ID  7): ( 0.XXX,  0.YYY,  0.ZZZ)
  Bottom-Right (ID 17): ( 0.XXX,  0.YYY,  0.ZZZ)
  Bottom-Left  (ID 11): ( 0.XXX,  0.YYY,  0.ZZZ)
  Workspace size: 0.XXXm x 0.YYYm
============================================================
```

**Check in RViz:**
- You should now see a **LIGHT BLUE SEMI-TRANSPARENT PLANE** between the 4 markers
- **BLUE BORDER LINES** around the workspace
- This is your calibrated workspace! ✅

**If calibration fails:**
```bash
# Check which markers are detected
rostopic echo /marker_detector/markers -n 1

# You should see all 4 markers (count: 4)
# If count < 4, adjust camera or marker positions
```

---

#### **STEP 6: Draw Line and Extract Trajectory**

**Physical Action:**
1. Using **BLUE PEN**, draw a line on the white paper
2. Draw **WITHIN** the workspace area (inside the 4 markers)
3. Can be straight line, curve, or any shape you want to test

**Terminal 3: Extract Line**
```bash
rosservice call /trajectory_tracker/extract_line
```

**Expected Output:**
```
success: True
message: "Line extracted! X points detected"
point_count: XX
```

**Check in RViz:**
You should now see **THREE visualizations**:

1. **RED POINTS** - Line boundary (contour of your drawn line)
   - Topic: `/trajectory_tracker/line_contour_markers`

2. **GREEN LINE** - Extracted trajectory (skeleton/centerline)
   - Topic: `/trajectory_tracker/line_skeleton_markers`

3. **YELLOW SPHERE** - Start point
4. **MAGENTA SPHERE** - End point

**Path visualization:**
- Green path should follow the centerline of your drawn line
- Should be smooth and continuous

**If line not detected:**
```bash
# Check line color detection (Terminal 3)
rosrun rqt_image_view rqt_image_view

# Select: /trajectory_tracker/extraction_debug
# You should see your line highlighted in green

# If not highlighted → adjust HSV color ranges in:
# ~/TUM/2025w/MSBRDM/welding_robot/src/trajectory_tracker/config/tracker_config.yaml
```

---

#### **STEP 7: Verify Complete System**

**Check all topics are publishing:**
```bash
# Terminal 4
source ~/TUM/2025w/MSBRDM/welding_robot/devel/setup.bash

# Check camera
rostopic hz /camera/image_raw
# Should show ~30 Hz

# Check markers
rostopic hz /marker_detector/markers
# Should show ~30 Hz

# Check extracted path
rostopic echo /trajectory_tracker/extracted_path -n 1
# Should show path with poses
```

**Check all nodes are running:**
```bash
rosnode list

# Should show:
# /camera_node
# /workspace_detector_node
# /line_extractor_node
# /trajectory_publisher_node
# /rviz
# /joint_state_publisher_ur10
# /robot_state_publisher_ur10
```

---

#### **STEP 8: Test Different Lines**

**Try various scenarios:**
1. **Straight line** - Test basic tracking
2. **Curved line** - Test smoothing
3. **Complex shape** - Test skeleton extraction
4. **Multiple lines** - Which one gets detected? (Should detect largest)

**For each test:**
1. Erase/replace paper with new line
2. Call: `rosservice call /trajectory_tracker/extract_line`
3. Observe in RViz

---

#### **STEP 9: Shutdown Properly**

**To close everything:**
```bash
# In Terminal 2 (where demo.launch is running):
Ctrl + C

# Wait for clean shutdown (should take ~2 seconds)
# All windows should close automatically

# If camera window doesn't close:
killall -9 python3

# Stop DroidCam (Terminal 1):
Ctrl + C
```

---

### Common Issues & Solutions

**Problem: "Cannot open camera device 0"**
```bash
# Solution:
# 1. Check DroidCam is running on phone
# 2. Reconnect USB cable
# 3. Run: droidcam-cli -a
# 4. Verify: ls /dev/video0
```

**Problem: "No markers detected"**
```bash
# Solutions:
# 1. Improve lighting
# 2. Check marker size is exactly 10cm (measure!)
# 3. Verify markers are FLAT on table
# 4. Move camera to better angle
# 5. Check /marker_detector/debug_image topic
```

**Problem: "Workspace calibration failed"**
```bash
# Solutions:
# 1. Ensure all 4 markers visible to camera
# 2. Check marker IDs match: 3, 7, 17, 11
# 3. Verify camera calibration received:
#    rostopic echo /camera/camera_info -n 1
```

**Problem: "Line not detected"**
```bash
# Solutions:
# 1. Use BLUE pen (not black/red)
# 2. Draw thicker line
# 3. Ensure line is within workspace area
# 4. Check color threshold in tracker_config.yaml
# 5. View /trajectory_tracker/extraction_debug
```

**Problem: "Computer freezes on shutdown"**
```bash
# Solution: Already fixed with shutdown hooks
# If still happens:
# 1. Ctrl + C in launch terminal
# 2. Wait 5 seconds
# 3. If frozen: Ctrl + Alt + F3, login, run: killall -9 python3
```

---

### Success Checklist ✅

After completing all steps, you should have:
- [x] Camera feed showing in window with marker detection overlays
- [x] RViz showing UR10 robot model
- [x] 4 blue spheres at marker positions in RViz
- [x] Light blue workspace plane in RViz
- [x] Red contour + green trajectory path in RViz after line extraction
- [x] Yellow (start) and magenta (end) spheres on trajectory
- [x] Clean shutdown without freezing

**If all checked → System working perfectly! 🎉**

---

### Tips for Best Results

1. **Lighting**: Bright, even lighting. Avoid shadows on markers.
2. **Camera Position**: Mount camera ~50-80cm above workspace, pointing straight down.
3. **Marker Quality**: Print on white paper with laser printer (not inkjet - can be blurry).
4. **Surface**: Use matte white paper (not glossy - causes reflections).
5. **Line Drawing**: Draw slowly with steady hand, consistent thickness.
6. **Calibration**: Recalibrate if you move camera or markers.

---

### Next Steps (Robot Control Integration)

Once trajectory extraction works reliably:
1. Connect to real UR10 robot
2. Integrate with motion controller
3. Send extracted path to robot
4. Test trajectory following in simulation (RViz)
5. Test on real robot with welding tool

**Current status**: Vision system ready, waiting for robot controller integration.
