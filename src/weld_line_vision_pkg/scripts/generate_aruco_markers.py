#!/usr/bin/env python3
"""
ArUco Marker Generation Script
- Workspace markers: ID 0,1,2,3 (6cm)
- Robot base markers: ID 7, 17 (10cm)
"""

import cv2
import numpy as np
import os

def generate_markers():
    # DICT_4X4_50 dictionary
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    # Create output directory
    output_dir = "aruco_markers"
    os.makedirs(output_dir, exist_ok=True)

    # Workspace markers (ID 0,1,2,3) - 6cm
    workspace_ids = [0, 1, 2, 3]
    workspace_size_mm = 60  # 6cm

    # Robot base markers (ID 7, 17) - 10cm
    robot_ids = [7, 17]
    robot_size_mm = 100  # 10cm

    # DPI setting (300 DPI recommended)
    dpi = 300

    print("=== ArUco Marker Generation ===\n")

    # Generate workspace markers
    print("Workspace Markers (6cm):")
    for marker_id in workspace_ids:
        # 6cm @ 300 DPI = 708 pixels
        marker_size_px = int(workspace_size_mm / 25.4 * dpi)

        img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)

        # Add margin (5mm)
        margin_px = int(5 / 25.4 * dpi)
        img_with_margin = cv2.copyMakeBorder(
            img, margin_px, margin_px, margin_px, margin_px,
            cv2.BORDER_CONSTANT, value=255
        )

        # Add text label
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = f"ID {marker_id} - 6cm"
        text_size = cv2.getTextSize(text, font, 1.5, 3)[0]
        text_x = (img_with_margin.shape[1] - text_size[0]) // 2
        text_y = img_with_margin.shape[0] - margin_px // 2
        cv2.putText(img_with_margin, text, (text_x, text_y),
                    font, 1.5, 0, 3, cv2.LINE_AA)

        filename = f"{output_dir}/workspace_marker_id{marker_id}_6cm.png"
        cv2.imwrite(filename, img_with_margin)
        print(f"  ✓ ID {marker_id}: {filename}")

    print("\nRobot Base Markers (10cm):")
    for marker_id in robot_ids:
        # 10cm @ 300 DPI = 1181 pixels
        marker_size_px = int(robot_size_mm / 25.4 * dpi)

        img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)

        # Add margin (8mm)
        margin_px = int(8 / 25.4 * dpi)
        img_with_margin = cv2.copyMakeBorder(
            img, margin_px, margin_px, margin_px, margin_px,
            cv2.BORDER_CONSTANT, value=255
        )

        # Add text label
        font = cv2.FONT_HERSHEY_SIMPLEX
        label = "TOP (Front)" if marker_id == 7 else "BOTTOM (Back)"
        text = f"ID {marker_id} - 10cm - {label}"
        text_size = cv2.getTextSize(text, font, 2, 4)[0]
        text_x = (img_with_margin.shape[1] - text_size[0]) // 2
        text_y = img_with_margin.shape[0] - margin_px // 2
        cv2.putText(img_with_margin, text, (text_x, text_y),
                    font, 2, 0, 4, cv2.LINE_AA)

        filename = f"{output_dir}/robot_marker_id{marker_id}_10cm.png"
        cv2.imwrite(filename, img_with_margin)
        print(f"  ✓ ID {marker_id}: {filename}")

    print(f"\nAll markers saved to '{output_dir}/' directory.")
    print("\nPrinting Instructions:")
    print("  1. Print at 300 DPI")
    print("  2. Select 'Actual Size' option")
    print("  3. Do not scale")
    print("\nPlacement:")
    print("  Workspace: ID 0(top-left), 1(top-right), 2(bottom-left), 3(bottom-right)")
    print("  Robot: ID 7(front/top), ID 17(back/bottom) - attach on same plane")

if __name__ == "__main__":
    generate_markers()
