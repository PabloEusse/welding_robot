#!/usr/bin/env python3
"""
ArUco Marker Generator
======================

Generates ArUco markers for workspace calibration.

Usage:
    python3 generate_aruco_markers.py

This will create marker images in the 'markers' directory.
Print them on white paper at the specified size.
"""

import cv2
import numpy as np
import os

# Marker configuration (matches tracker_config.yaml)
ARUCO_DICT_NAME = "DICT_4X4_50"
MARKER_SIZE_CM = 10  # 10cm for better detection
MARKER_IDS = [3, 7, 17, 11]  # For workspace calibration (4 corners: TL, TR, BR, BL)

# Image generation settings
MARKER_SIZE_PIXELS = 200  # Resolution of generated image
BORDER_SIZE_PIXELS = 20   # White border around marker
DPI = 300  # Print resolution

def main():
    # Create output directory
    output_dir = "markers"
    os.makedirs(output_dir, exist_ok=True)

    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    print("=" * 60)
    print("ArUco Marker Generator")
    print("=" * 60)
    print(f"Dictionary: {ARUCO_DICT_NAME}")
    print(f"Marker size: {MARKER_SIZE_CM}cm x {MARKER_SIZE_CM}cm")
    print(f"Marker IDs: {MARKER_IDS}")
    print(f"Output directory: {output_dir}/")
    print("=" * 60)
    print()

    for marker_id in MARKER_IDS:
        # Generate marker image
        marker_image = cv2.aruco.generateImageMarker(
            aruco_dict,
            marker_id,
            MARKER_SIZE_PIXELS
        )

        # Add white border for easier detection
        bordered_image = cv2.copyMakeBorder(
            marker_image,
            BORDER_SIZE_PIXELS,
            BORDER_SIZE_PIXELS,
            BORDER_SIZE_PIXELS,
            BORDER_SIZE_PIXELS,
            cv2.BORDER_CONSTANT,
            value=255
        )

        # Add label text
        label_height = 40
        full_image = np.ones((bordered_image.shape[0] + label_height,
                              bordered_image.shape[1]), dtype=np.uint8) * 255
        full_image[label_height:, :] = bordered_image

        # Add ID label
        cv2.putText(
            full_image,
            f"ID: {marker_id}  |  Size: {MARKER_SIZE_CM}cm",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            0,
            2
        )

        # Save marker
        filename = f"{output_dir}/aruco_marker_{marker_id}.png"
        cv2.imwrite(filename, full_image)
        print(f"✓ Generated: {filename}")

    print()
    print("=" * 60)
    print("PRINTING INSTRUCTIONS")
    print("=" * 60)
    print(f"1. Print the markers at {MARKER_SIZE_CM}cm x {MARKER_SIZE_CM}cm")
    print("   - Use high-quality white paper")
    print("   - Print at 100% scale (no scaling)")
    print("   - Measure with a ruler to verify size!")
    print()
    print("2. Mount markers at 4 corners of workspace:")
    print()
    print("   Layout (looking from above):")
    print()
    print("        ID 3 (TL) ----------- ID 7 (TR)")
    print("           |                      |")
    print("           |     WORKSPACE        |")
    print("           |                      |")
    print("       ID 11 (BL) ---------- ID 17 (BR)")
    print()
    print("   TL=Top-Left, TR=Top-Right, BR=Bottom-Right, BL=Bottom-Left")
    print()
    print("3. Marker placement tips:")
    print("   - Keep markers FLAT and PARALLEL to workspace surface")
    print("   - Place outside the drawing area (at corners)")
    print("   - Ensure all 4 markers are visible to camera")
    print("   - Avoid shadows or reflections on markers")
    print()
    print("4. Verify detection:")
    print("   roslaunch trajectory_tracker trajectory_tracker.launch show_gui:=true")
    print("   - Check /marker_detector/debug_image topic")
    print("   - You should see all 4 marker IDs and axes drawn")
    print()
    print("5. Calibrate workspace:")
    print("   rosservice call /trajectory_tracker/calibrate_workspace \"{marker_ids: [3, 7, 17, 11]}\"")
    print("=" * 60)

if __name__ == '__main__':
    main()
