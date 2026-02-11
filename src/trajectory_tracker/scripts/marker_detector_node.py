#!/usr/bin/env python3
"""
Marker Detector Node
====================

Detects ArUco markers or colored pen tips in camera images.
Publishes detected pixel coordinates for coordinate transformation.

Detection Modes:
1. ARUCO: Detect ArUco markers (recommended for accuracy)
2. COLOR: Detect colored pen tip by HSV color filtering

Topics Subscribed:
    /camera/image_raw (sensor_msgs/Image): Input camera image

Topics Published:
    /marker_detector/detection (trajectory_tracker/TrajectoryTarget): Detected marker
    /marker_detector/debug_image (sensor_msgs/Image): Debug visualization

Parameters:
    ~detection_mode (str): "aruco" or "color" (default: "aruco")
    ~aruco_dict (str): ArUco dictionary (default: "DICT_4X4_50")
    ~marker_size (float): ArUco marker size in meters (default: 0.05)
    ~color_lower (list): Lower HSV bound for color detection
    ~color_upper (list): Upper HSV bound for color detection
    ~min_area (int): Minimum contour area for color detection (default: 100)
    ~show_debug (bool): Publish debug image (default: True)
"""

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from trajectory_tracker.msg import TrajectoryTarget


class MarkerDetectorNode:
    # ArUco dictionary mapping
    ARUCO_DICTS = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    }

    def __init__(self):
        rospy.init_node('marker_detector_node')

        # Parameters
        self.detection_mode = rospy.get_param('~detection_mode', 'aruco')
        aruco_dict_name = rospy.get_param('~aruco_dict', 'DICT_4X4_50')
        self.marker_size = rospy.get_param('~marker_size', 0.05)  # 5cm default

        # Color detection parameters (default: red pen)
        # HSV ranges for red color (red wraps around 0/180)
        self.color_lower1 = np.array(rospy.get_param('~color_lower1', [0, 100, 100]))
        self.color_upper1 = np.array(rospy.get_param('~color_upper1', [10, 255, 255]))
        self.color_lower2 = np.array(rospy.get_param('~color_lower2', [160, 100, 100]))
        self.color_upper2 = np.array(rospy.get_param('~color_upper2', [180, 255, 255]))

        self.min_area = rospy.get_param('~min_area', 100)
        self.show_debug = rospy.get_param('~show_debug', True)

        # CV Bridge
        self.bridge = CvBridge()

        # ArUco setup
        if aruco_dict_name in self.ARUCO_DICTS:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICTS[aruco_dict_name])
        else:
            rospy.logwarn(f"Unknown ArUco dict {aruco_dict_name}, using DICT_4X4_50")
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.aruco_params = cv2.aruco.DetectorParameters()

        # Camera info storage
        self.camera_matrix = None
        self.dist_coeffs = None

        # Publishers
        self.detection_pub = rospy.Publisher(
            '/marker_detector/detection', TrajectoryTarget, queue_size=1)
        self.debug_image_pub = rospy.Publisher(
            '/marker_detector/debug_image', Image, queue_size=1)

        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            '/camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        rospy.loginfo(f"Marker detector started in {self.detection_mode.upper()} mode")

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration data."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera info received")

    def image_callback(self, msg: Image):
        """Process incoming image and detect markers."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to convert image: {e}")
            return

        # Detect based on mode
        if self.detection_mode == 'aruco':
            detection, debug_frame = self.detect_aruco(frame)
        else:
            detection, debug_frame = self.detect_color(frame)

        # Publish detection
        if detection is not None:
            detection.header.stamp = msg.header.stamp
            detection.header.frame_id = "camera_link"
            self.detection_pub.publish(detection)

        # Publish debug image
        if self.show_debug and debug_frame is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Failed to publish debug image: {e}")

    def detect_aruco(self, frame: np.ndarray):
        """
        Detect ArUco markers in the frame.
        Returns the center of the first detected marker.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug_frame = frame.copy()

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        detection = TrajectoryTarget()
        detection.valid = False
        detection.marker_id = -1
        detection.confidence = 0.0

        if ids is not None and len(ids) > 0:
            # Draw all detected markers
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            # Use the first detected marker
            marker_corners = corners[0][0]  # Shape: (4, 2)
            marker_id = ids[0][0]

            # Calculate center of marker
            center_x = np.mean(marker_corners[:, 0])
            center_y = np.mean(marker_corners[:, 1])

            # Estimate pose if camera is calibrated
            if self.camera_matrix is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                # Draw axis on debug image
                cv2.drawFrameAxes(debug_frame, self.camera_matrix, self.dist_coeffs,
                                  rvecs[0], tvecs[0], self.marker_size * 0.5)

                # Store 3D position from marker pose (in camera frame)
                detection.position.x = tvecs[0][0][0]
                detection.position.y = tvecs[0][0][1]
                detection.position.z = tvecs[0][0][2]

            # Fill detection message
            detection.valid = True
            detection.marker_id = int(marker_id)
            detection.pixel_x = center_x
            detection.pixel_y = center_y
            detection.confidence = 1.0

            # Draw center point
            cv2.circle(debug_frame, (int(center_x), int(center_y)), 10, (0, 255, 0), -1)

            # Display info
            info_text = f"ID:{marker_id} ({center_x:.0f}, {center_y:.0f})"
            cv2.putText(debug_frame, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(debug_frame, "No marker detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return detection, debug_frame

    def detect_color(self, frame: np.ndarray):
        """
        Detect colored pen tip using HSV color filtering.
        Returns the center of the largest detected blob.
        """
        debug_frame = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask for color (handle red wrap-around)
        mask1 = cv2.inRange(hsv, self.color_lower1, self.color_upper1)
        mask2 = cv2.inRange(hsv, self.color_lower2, self.color_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detection = TrajectoryTarget()
        detection.valid = False
        detection.marker_id = -1  # -1 indicates color detection, not ArUco
        detection.confidence = 0.0

        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area >= self.min_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = M["m10"] / M["m00"]
                    center_y = M["m01"] / M["m00"]

                    # Fill detection message
                    detection.valid = True
                    detection.pixel_x = center_x
                    detection.pixel_y = center_y
                    detection.confidence = min(1.0, area / 1000.0)  # Confidence based on area

                    # Draw detection
                    cv2.drawContours(debug_frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (int(center_x), int(center_y)), 10, (0, 255, 0), -1)

                    info_text = f"Pen ({center_x:.0f}, {center_y:.0f}) area:{area:.0f}"
                    cv2.putText(debug_frame, info_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if not detection.valid:
            cv2.putText(debug_frame, "No pen detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Show mask in corner for debugging
        mask_small = cv2.resize(mask, (160, 120))
        mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        debug_frame[0:120, 0:160] = mask_bgr

        return detection, debug_frame


def main():
    try:
        node = MarkerDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
