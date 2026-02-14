#!/usr/bin/env python3
"""
Print startup instructions for trajectory tracker demo.
"""

import rospy
import time

def main():
    rospy.init_node('instructions', anonymous=True)
    time.sleep(3)  # Wait for other nodes to start

    print()
    print("=" * 70)
    print("TRAJECTORY TRACKER DEMO")
    print("=" * 70)
    print()
    print("✓ UR10 Robot loaded")
    print("✓ Camera node started")
    print("✓ Marker detector running")
    print("✓ RViz opened")
    print()
    print("=" * 70)
    print("NEXT STEPS")
    print("=" * 70)
    print()
    print("1. Place 4 ArUco markers at workspace corners:")
    print()
    print("        ID 3 (TL) ----------- ID 7 (TR)")
    print("           |                      |")
    print("           |     WORKSPACE        |")
    print("           |                      |")
    print("       ID 11 (BL) ---------- ID 17 (BR)")
    print()
    print("2. Calibrate workspace:")
    print()
    print("   rosservice call /trajectory_tracker/calibrate_workspace \\")
    print("     \"{marker_ids: [3, 7, 17, 11]}\"")
    print()
    print("3. Draw a BLUE line on paper within workspace")
    print()
    print("4. Extract trajectory:")
    print()
    print("   rosservice call /trajectory_tracker/extract_line")
    print()
    print("=" * 70)
    print()
    print("Check RViz to see:")
    print("  - Robot model (grey)")
    print("  - ArUco marker axes (RGB)")
    print("  - Workspace plane (light blue)")
    print("  - Line contour (red)")
    print("  - Trajectory skeleton (green)")
    print()
    print("=" * 70)
    print()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
