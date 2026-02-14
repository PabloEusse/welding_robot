# Visual Check Guide - Complete System Test

## Robot Base Frame Definition (CORRECTED)

**Origin:** Center of marker 7
**Z-axis (Blue):** Points from marker 17 → marker 7 (upward)
**Y-axis (Green):** Points left (perpendicular to Z)
**X-axis (Red):** Points forward (into workspace, by right-hand rule = Y × Z)

```
Visualization:
        Z↑ (blue, from 17→7)
         |
    [ID 7] ← origin
       / \
      /   \
     Y     (X points into workspace)
   (left)

[ID 17] (below, for Z-axis orientation reference)
```

---

## Quick Launch & Verification

### 1. One-Command Launch
```bash
# Open terminal
cd ~/welding_robot
source devel/setup.bash
roslaunch weld_line_vision vision_with_robot_rviz.launch
```

This single command launches:
- ✅ USB camera (DroidCam)
- ✅ Vision processing node
- ✅ Robot model (UR10 + gripper)
- ✅ Joint state publisher GUI
- ✅ RViz with full visualization

---

## 2. What You Should See Immediately

### RViz Window (Main 3D View):

**Expected elements:**
1. **Grid** - Gray grid on XY plane (world frame)
2. **TF Axes** - Colored arrows showing coordinate frames
   - Red = X-axis
   - Green = Y-axis
   - Blue = Z-axis
3. **Green Rectangle** - Workspace boundary (if markers 0,1,2,3 detected)
4. **Robot Model** - UR10 arm + gripper (if markers 7,17 detected)
5. **Red Path** - Weld trajectory (if cyan/blue line detected)
6. **Blue Spheres** - Sampled weld points

### Debug Image Window (Camera View):

**Expected elements:**
- Camera feed with overlays
- **Yellow squares** around detected ArUco markers
- **Yellow circles** at marker centers with IDs ("TL", "TR", "BL", "BR")
- **Green pixels** showing detected weld line skeleton
- **Red dots** on sampled trajectory points

### Joint State Publisher GUI:

**Expected elements:**
- Sliders for each robot joint
- Move sliders to see robot move in RViz

---

## 3. Frame Coordinate Verification

### Check TF Tree:
```bash
# In a new terminal
source ~/welding_robot/devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree
```

**Expected TF tree structure:**
```
world
  └─ workspace_frame
       └─ base_link (origin at marker 7)
            └─ shoulder_link
                 └─ upper_arm_link
                      └─ forearm_link
                           └─ wrist_1_link
                                └─ wrist_2_link
                                     └─ wrist_3_link
                                          └─ ee_link
                                               └─ gripper_base_link
                                                    └─ ... (gripper fingers)
```

### Verify Robot Base Frame (at marker 7):

**In RViz, look at the TF frame at marker 7:**
1. **Origin**: Should be exactly at marker 7 center
2. **Z-axis (Blue)**: Should point from marker 17 → marker 7
3. **Y-axis (Green)**: Should point left (90° CCW from Z in the workspace plane)
4. **X-axis (Red)**: Should point forward/into workspace (perpendicular to both Y and Z)

**Frame orientation diagram:**
```
In workspace plane view:

  17→7 direction defines Z-axis
  Y is perpendicular (left)
  X points out of/into plane

     workspace
        ↑ Y
        |
  ------+-----→ X
        |
        Z (17→7)
```

---

## 4. Terminal Output Verification

### Expected Console Messages:

```
[ INFO] Trajectory Generator initialized. Subscribing to: /droidcam/image_raw
[ INFO] ArUco ids: TL=0 TR=1 BL=2 BR=3 (dict=10)
[ INFO] Robot base at marker 7: pos=(0.105, 0.148, 0.000)m, Z-axis: 17→7(45.2°)
```

**What this means:**
- ✅ Vision node started
- ✅ Workspace markers configured (0,1,2,3)
- ✅ Robot base detected at marker 7
- ✅ Position in workspace frame (meters)
- ✅ Z-axis direction angle (degrees) - angle of vector 17→7 in workspace XY plane

### If markers NOT detected:
```
[ WARN] Missing ArUco corners. Need ids TL=0 TR=1 BL=2 BR=3
[ WARN] Robot markers not detected. Need IDs 7 (top) and 17 (bottom)
```

---

## 5. ROS Topics Check

### List active topics:
```bash
rostopic list | grep weld_line_vision
```

**Expected output:**
```
/weld_line_vision/debug_image
/weld_line_vision/debug_mask
/weld_line_vision/debug_skeleton
/weld_line_vision/weld_path
/weld_line_vision/weld_points_marker
/weld_line_vision/workspace_marker
```

### Echo robot position:
```bash
# In new terminal
rostopic echo /tf --filter "transforms[0].child_frame_id == 'base_link'"
```

**Expected output:**
```yaml
transforms:
  - header:
      frame_id: "workspace_frame"
    child_frame_id: "base_link"
    transform:
      translation:
        x: 0.105  # Marker 7 X position
        y: 0.148  # Marker 7 Y position
        z: 0.0    # On workspace plane
      rotation:
        # Quaternion representing the 3D rotation
        # Z-axis points from 17→7
```

---

## 6. Interactive Tests

### Test 1: Move Robot Joints
1. In Joint State Publisher GUI, move sliders
2. Watch robot move in RViz
3. **Expected**: Smooth motion, robot base stays at marker 7

### Test 2: Check Workspace Boundary
1. In RViz, look for green rectangle
2. **Expected**: Rectangle corners align with markers 0,1,2,3

### Test 3: Verify Robot Position & Orientation
1. Observe robot base position (should be at marker 7)
2. Check Z-axis (blue) aligns with 17→7 direction
3. Move markers slightly and watch robot update
4. **Expected**: Robot base follows marker 7, Z-axis follows 17→7 vector

### Test 4: Weld Line Detection
1. Place cyan/blue tape or marker on workspace
2. **Expected**: Red path appears following the line

---

## 7. Visual Inspection Checklist

### ✅ Workspace Frame:
- [ ] Green rectangle visible
- [ ] Rectangle matches A4 paper size
- [ ] Corners align with markers 0,1,2,3

### ✅ Robot Base Frame:
- [ ] Robot appears in RViz
- [ ] Base origin is at marker 7 position (exactly)
- [ ] Z-axis (blue) points from 17→7
- [ ] Y-axis (green) points left (perpendicular to Z)
- [ ] X-axis (red) points forward/into workspace

### ✅ Weld Trajectory:
- [ ] Red path visible (if weld line present)
- [ ] Blue points along trajectory
- [ ] Path follows detected line

### ✅ TF Frames:
- [ ] All axes visible
- [ ] Frame hierarchy correct (world→workspace→base_link→robot)
- [ ] No errors in TF tree

---

## 8. Troubleshooting Quick Fixes

### Problem: No workspace rectangle
**Fix:**
```bash
# Check if all 4 markers detected
rostopic echo /weld_line_vision/debug_image
# Look for "TL", "TR", "BL", "BR" labels
```

### Problem: No robot model
**Fix:**
```bash
# Check if markers 7 and 17 detected
# Verify in terminal output or debug image

# Check robot description loaded
rosparam get /robot_description | head -20
```

### Problem: Robot at wrong position
**Fix:**
- Verify marker 7 is at expected location
- Check marker 7 is labeled "ID 7" in debug image
- Confirm origin is exactly at marker 7 center

### Problem: Robot orientation wrong
**Fix:**
- Verify marker 17 is positioned correctly relative to marker 7
- Check Z-axis (blue) points from 17→7 in RViz
- Verify 17→7 vector direction matches expected robot "up" direction
- Adjust marker placement if needed

### Problem: Z-axis not pointing from 17→7
**Fix:**
- Confirm both markers 7 and 17 are detected (check debug image)
- Verify markers are not too close (should be >5cm apart)
- Check console for "Markers 7 and 17 are too close!" warning

---

## 9. Save RViz Configuration

If you adjust view in RViz:
```
File → Save Config As...
Save to: ~/welding_robot/src/weld_line_vision_pkg/config/robot_workspace.rviz
```

---

## 10. Record Test Data

```bash
# Record all relevant topics
rosbag record /weld_line_vision/weld_path /tf /joint_states -O test_session.bag

# Playback later
rosbag play test_session.bag
```

---

## Summary: Success Indicators

**✅ System Working Correctly:**
1. RViz shows robot model
2. Robot base origin at marker 7 (exact position)
3. Z-axis (blue) points from marker 17 → marker 7
4. Y-axis (green) points left
5. X-axis (red) points forward
6. Green workspace rectangle visible
7. TF tree complete (world→workspace→base_link→robot)
8. Console shows "Robot base at marker 7: ..." message
9. Robot moves when joint sliders adjusted

**If ALL above are true** → **System is fully operational!** 🎉

---

## Quick Reference Commands

```bash
# Launch everything
roslaunch weld_line_vision vision_with_robot_rviz.launch

# Check TF tree (new terminal)
rosrun rqt_tf_tree rqt_tf_tree

# View camera feed
rosrun image_view image_view image:=/weld_line_vision/debug_image

# Monitor robot position
rostopic echo /tf | grep -A 10 base_link

# List all topics
rostopic list

# Check node status
rosnode list
rosnode info /weld_line_vision
```

---

**Ready to test!** Just run the launch command and follow this checklist. 🚀
