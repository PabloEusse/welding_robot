#!/usr/bin/env python3
"""
Workspace Detector Node
========================

Detects ArUco markers and transforms coordinates to robot base frame.
Provides workspace calibration using 4 corner markers.

Topics Subscribed:
    /camera/image_raw (sensor_msgs/Image): Input camera image
    /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration

Topics Published:
    /marker_detector/detection (trajectory_tracker/TrajectoryTarget): Single marker (backward compatibility)
    /marker_detector/markers (trajectory_tracker/MarkerArray): All detected markers
    /marker_detector/debug_image (sensor_msgs/Image): Debug visualization
    /trajectory_tracker/target (trajectory_tracker/TrajectoryTarget): Target in robot base frame
    /trajectory_tracker/workspace (trajectory_tracker/WorkspaceCalibration): Workspace info
    /trajectory_tracker/marker_visualization (visualization_msgs/MarkerArray): RViz markers
    /trajectory_tracker/workspace_plane (visualization_msgs/MarkerArray): Workspace plane

Services:
    /trajectory_tracker/get_target (GetTargetPosition): Get current target position
    /trajectory_tracker/calibrate_workspace (CalibrateWorkspace): Calibrate workspace corners

Parameters:
    ~aruco_dict (str): ArUco dictionary (default: "DICT_4X4_50")
    ~marker_size (float): ArUco marker size in meters (default: 0.10)
    ~show_debug (bool): Publish debug image (default: True)
    ~camera_to_robot (list): [x, y, z, roll, pitch, yaw] camera pose in robot frame
"""

import rospy
import cv2
import numpy as np
import tf
import tf.transformations as tft
from threading import Lock

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray as VizMarkerArray

from trajectory_tracker.msg import TrajectoryTarget, MarkerArray, WorkspaceCalibration
from trajectory_tracker.srv import (
    GetTargetPosition, GetTargetPositionResponse,
    CalibrateWorkspace, CalibrateWorkspaceResponse
)


# ============================================================
# SECTION 1: WORKSPACE DETECTOR NODE
# ============================================================

class WorkspaceDetectorNode:
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
        rospy.init_node('workspace_detector_node')

        # Shutdown flag
        self.is_shutdown = False

        # ============================================================
        # SECTION 1.1: ARUCO DETECTION PARAMETERS
        # ============================================================
        aruco_dict_name = rospy.get_param('~aruco_dict', 'DICT_4X4_50')
        self.marker_size = rospy.get_param('~marker_size', 0.10)  # 10cm default
        self.show_debug = rospy.get_param('~show_debug', True)

        # ArUco setup with version compatibility
        if aruco_dict_name in self.ARUCO_DICTS:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICTS[aruco_dict_name])
        else:
            rospy.logwarn(f"Unknown ArUco dict {aruco_dict_name}, using DICT_4X4_50")
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # OpenCV version compatibility
        self.use_legacy_api = False
        try:
            # Try new API (OpenCV 4.7+)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            rospy.loginfo("[WorkspaceDetector] Using OpenCV 4.7+ ArUco API")
        except AttributeError:
            # Fallback to legacy API
            try:
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.aruco_detector = None
                self.use_legacy_api = True
                rospy.loginfo("[WorkspaceDetector] Using legacy OpenCV ArUco API")
            except Exception as e:
                rospy.logfatal(f"Failed to initialize ArUco detector: {e}")
                raise

        # ============================================================
        # SECTION 1.2: COORDINATE TRANSFORMATION PARAMETERS
        # ============================================================
        # Camera to robot base transformation [x, y, z, roll, pitch, yaw]
        cam_pose = rospy.get_param('~camera_to_robot', [0.5, 0.0, 0.8, 3.14159, 0.0, 0.0])
        self.T_robot_camera = self._pose_to_matrix(cam_pose)

        # Camera intrinsics (will be populated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Current detection storage
        self.current_target = None
        self.target_lock = Lock()

        # ============================================================
        # SECTION 1.3: WORKSPACE CALIBRATION STATE
        # ============================================================
        self.workspace_calibrated = False
        self.workspace_marker_ids = [3, 7, 17, 11]  # Default: TL, TR, BR, BL
        self.workspace_corners = []  # Robot frame positions of 4 corners
        self.workspace_markers = {}  # Store detected markers {id: TrajectoryTarget}

        # CV Bridge
        self.bridge = CvBridge()

        # ============================================================
        # SECTION 1.4: PUBLISHERS
        # ============================================================
        # Marker detection publishers
        self.detection_pub = rospy.Publisher(
            '/marker_detector/detection', TrajectoryTarget, queue_size=1)
        self.marker_array_pub = rospy.Publisher(
            '/marker_detector/markers', MarkerArray, queue_size=1)
        self.debug_image_pub = rospy.Publisher(
            '/marker_detector/debug_image', Image, queue_size=1)

        # Coordinate transformation publishers
        self.target_pub = rospy.Publisher(
            '/trajectory_tracker/target', TrajectoryTarget, queue_size=1)
        self.workspace_pub = rospy.Publisher(
            '/trajectory_tracker/workspace', WorkspaceCalibration, queue_size=1)

        # Visualization publishers
        self.marker_viz_pub = rospy.Publisher(
            '/trajectory_tracker/marker_visualization', VizMarkerArray, queue_size=1)
        self.workspace_plane_pub = rospy.Publisher(
            '/trajectory_tracker/workspace_plane', VizMarkerArray, queue_size=1)

        # ============================================================
        # SECTION 1.5: SUBSCRIBERS
        # ============================================================
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            '/camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        # ============================================================
        # SECTION 1.6: SERVICES
        # ============================================================
        self.get_target_srv = rospy.Service(
            '/trajectory_tracker/get_target', GetTargetPosition, self.get_target_service)
        self.calibrate_workspace_srv = rospy.Service(
            '/trajectory_tracker/calibrate_workspace', CalibrateWorkspace, self.calibrate_workspace_service)

        # TF broadcaster for visualization
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Workspace detector node started")

    def shutdown(self):
        """Clean up resources on shutdown."""
        rospy.loginfo("Shutting down workspace detector node...")
        self.is_shutdown = True
        rospy.loginfo("Workspace detector node shutdown complete")

    # ============================================================
    # SECTION 2: ARUCO MARKER DETECTION
    # ============================================================

    def image_callback(self, msg: Image):
        """Process incoming image and detect ArUco markers."""
        if self.is_shutdown:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to convert image: {e}")
            return

        # Detect ArUco markers
        detection, marker_array, debug_frame = self.detect_aruco(frame)

        # Publish single detection (for backward compatibility)
        if detection is not None:
            detection.header.stamp = msg.header.stamp
            detection.header.frame_id = "camera_link"
            self.detection_pub.publish(detection)

            # Transform to robot frame and publish
            self._transform_and_publish_target(detection, msg.header.stamp)

        # Publish marker array (for multi-marker support)
        if marker_array is not None:
            marker_array.header.stamp = msg.header.stamp
            marker_array.header.frame_id = "camera_link"
            self.marker_array_pub.publish(marker_array)

            # Store markers for workspace calibration
            self._store_markers(marker_array)

            # Visualize markers in RViz
            if len(marker_array.markers) > 0:
                self._publish_marker_visualization(marker_array.markers)

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
        Returns: (single_detection, marker_array, debug_frame)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug_frame = frame.copy()

        # Detect markers - version compatibility
        if self.use_legacy_api:
            # Legacy API (OpenCV < 4.7)
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)
        else:
            # New API (OpenCV >= 4.7)
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        detection = TrajectoryTarget()
        detection.valid = False
        detection.marker_id = -1
        detection.confidence = 0.0

        # Create marker array message
        marker_array = MarkerArray()
        marker_array.header.stamp = rospy.Time.now()
        marker_array.header.frame_id = "camera"
        marker_array.count = 0

        if ids is not None and len(ids) > 0:
            # Draw all detected markers
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            marker_array.count = len(ids)

            # Process ALL detected markers
            for i, (corner, marker_id) in enumerate(zip(corners, ids)):
                marker_corners = corner[0]  # Shape: (4, 2)
                marker_id = marker_id[0]

                # Calculate center of marker
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])

                # Create detection for this marker
                marker_detection = TrajectoryTarget()
                marker_detection.header.stamp = rospy.Time.now()
                marker_detection.header.frame_id = "camera"
                marker_detection.valid = True
                marker_detection.marker_id = int(marker_id)
                marker_detection.pixel_x = center_x
                marker_detection.pixel_y = center_y
                marker_detection.confidence = 1.0

                # Estimate pose if camera is calibrated
                if self.camera_matrix is not None:
                    try:
                        # Try legacy API first (estimatePoseSingleMarkers)
                        if self.use_legacy_api or hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
                            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                                [corner], self.marker_size, self.camera_matrix, self.dist_coeffs)
                        else:
                            # New API: use cv2.solvePnP with marker corners
                            half_size = self.marker_size / 2.0
                            objPoints = np.array([
                                [-half_size,  half_size, 0],  # Top-left
                                [ half_size,  half_size, 0],  # Top-right
                                [ half_size, -half_size, 0],  # Bottom-right
                                [-half_size, -half_size, 0]   # Bottom-left
                            ], dtype=np.float32)

                            success, rvec, tvec = cv2.solvePnP(
                                objPoints, marker_corners,
                                self.camera_matrix, self.dist_coeffs,
                                flags=cv2.SOLVEPNP_IPPE_SQUARE
                            )
                            if success:
                                rvecs = [rvec]
                                tvecs = [tvec]
                            else:
                                raise RuntimeError("solvePnP failed")

                        # Draw axis on debug image
                        try:
                            if hasattr(cv2, 'drawFrameAxes'):
                                cv2.drawFrameAxes(debug_frame, self.camera_matrix, self.dist_coeffs,
                                                rvecs[0], tvecs[0], self.marker_size * 0.5)
                            else:
                                cv2.aruco.drawAxis(debug_frame, self.camera_matrix, self.dist_coeffs,
                                                 rvecs[0], tvecs[0], self.marker_size * 0.5)
                        except Exception as e:
                            rospy.logwarn_throttle(5.0, f"Failed to draw marker axis: {e}")

                        # Store 3D position from marker pose (in camera frame)
                        marker_detection.position.x = tvecs[0][0][0] if len(tvecs[0].shape) > 1 else tvecs[0][0]
                        marker_detection.position.y = tvecs[0][0][1] if len(tvecs[0].shape) > 1 else tvecs[0][1]
                        marker_detection.position.z = tvecs[0][0][2] if len(tvecs[0].shape) > 1 else tvecs[0][2]

                    except Exception as e:
                        rospy.logwarn_throttle(2.0, f"Failed to estimate marker pose: {e}")

                # Add to marker array
                marker_array.markers.append(marker_detection)

                # Draw center point
                cv2.circle(debug_frame, (int(center_x), int(center_y)), 10, (0, 255, 0), -1)

                # Display info for each marker
                info_text = f"ID:{marker_id}"
                cv2.putText(debug_frame, info_text,
                           (int(center_x) - 20, int(center_y) - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Use the first detected marker for backward compatibility
            detection = marker_array.markers[0]

            # Summary text
            summary = f"Detected {len(ids)} markers"
            cv2.putText(debug_frame, summary, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(debug_frame, "No marker detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return detection, marker_array, debug_frame

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration data."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera calibration received")

    # ============================================================
    # SECTION 3: COORDINATE TRANSFORMATION
    # ============================================================

    def _pose_to_matrix(self, pose):
        """Convert [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix."""
        x, y, z, roll, pitch, yaw = pose
        T = tft.euler_matrix(roll, pitch, yaw)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def _transform_and_publish_target(self, detection: TrajectoryTarget, timestamp):
        """Transform detected marker to robot base frame and publish."""
        if self.camera_matrix is None:
            rospy.logwarn_throttle(2.0, "Waiting for camera calibration...")
            return

        if not detection.valid:
            return

        # Transform marker pose to robot frame
        position = self._marker_pose_to_robot(detection)

        if position is None:
            return

        # Create output message
        target = TrajectoryTarget()
        target.header.stamp = timestamp
        target.header.frame_id = "base_link"
        target.position.x = position[0]
        target.position.y = position[1]
        target.position.z = position[2]
        target.pixel_x = detection.pixel_x
        target.pixel_y = detection.pixel_y
        target.confidence = detection.confidence
        target.marker_id = detection.marker_id
        target.valid = True

        # Store and publish
        with self.target_lock:
            self.current_target = target

        self.target_pub.publish(target)

        # Broadcast TF for visualization
        self.tf_broadcaster.sendTransform(
            (position[0], position[1], position[2]),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "trajectory_target",
            "base_link"
        )

    def _marker_pose_to_robot(self, msg: TrajectoryTarget):
        """
        Transform marker 3D position from camera frame to robot frame.
        Uses ArUco pose estimation results.
        """
        # Position from marker detection (in camera frame)
        p_camera = np.array([msg.position.x, msg.position.y, msg.position.z, 1.0])

        # Transform to robot frame
        p_robot = self.T_robot_camera @ p_camera

        return p_robot[:3]

    def _camera_to_robot_transform(self, point_camera):
        """Transform a point from camera frame to robot frame."""
        p_camera = np.array([point_camera.x, point_camera.y, point_camera.z, 1.0])
        p_robot = self.T_robot_camera @ p_camera
        return p_robot[:3]

    # ============================================================
    # SECTION 4: WORKSPACE CALIBRATION
    # ============================================================

    def _store_markers(self, marker_array: MarkerArray):
        """Store all detected markers for workspace calibration."""
        for marker in marker_array.markers:
            if marker.valid:
                self.workspace_markers[marker.marker_id] = marker

    def calibrate_workspace_service(self, req):
        """
        Calibrate workspace using four corner markers.

        Marker placement order (looking at workspace from above):
            marker_ids[0] (TL) -------- marker_ids[1] (TR)
                 |                              |
                 |         Workspace            |
                 |                              |
            marker_ids[3] (BL) -------- marker_ids[2] (BR)

        TL=Top-Left, TR=Top-Right, BR=Bottom-Right, BL=Bottom-Left
        """
        response = CalibrateWorkspaceResponse()
        response.workspace = WorkspaceCalibration()
        response.workspace.header.stamp = rospy.Time.now()
        response.workspace.header.frame_id = "base_link"

        # Use requested marker IDs or defaults [TL, TR, BR, BL]
        marker_ids = req.marker_ids if len(req.marker_ids) == 4 else self.workspace_marker_ids

        if len(marker_ids) != 4:
            response.success = False
            response.message = "Need exactly 4 marker IDs: [TL, TR, BR, BL]"
            response.workspace.is_calibrated = False
            return response

        rospy.loginfo(f"Calibrating workspace using markers {marker_ids}...")
        rospy.loginfo(f"  Order: TL={marker_ids[0]}, TR={marker_ids[1]}, BR={marker_ids[2]}, BL={marker_ids[3]}")

        # Check if all 4 markers are detected
        for i, marker_id in enumerate(marker_ids):
            if marker_id not in self.workspace_markers:
                corner_names = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
                response.success = False
                response.message = f"Marker ID {marker_id} ({corner_names[i]}) not detected! Place it in camera view."
                response.workspace.is_calibrated = False
                return response

        # Get all 4 markers
        markers = [self.workspace_markers[mid] for mid in marker_ids]

        # Transform all marker positions to robot frame
        try:
            corner_positions = []

            for marker in markers:
                # Use 3D position from ArUco pose estimation
                if marker.position.x != 0 or marker.position.y != 0 or marker.position.z != 0:
                    pos_robot = self._camera_to_robot_transform(marker.position)
                else:
                    rospy.logwarn(f"Marker {marker.marker_id} has no 3D pose! Ensure camera is calibrated.")
                    response.success = False
                    response.message = "Markers must have 3D pose (check camera calibration)"
                    response.workspace.is_calibrated = False
                    return response

                corner_positions.append(pos_robot)

            # Store workspace calibration
            self.workspace_corners = corner_positions
            self.workspace_marker_ids = marker_ids
            self.workspace_calibrated = True

            # Calculate workspace dimensions
            width = np.linalg.norm(corner_positions[1] - corner_positions[0])
            height = np.linalg.norm(corner_positions[3] - corner_positions[0])

            # Fill response
            response.success = True
            response.message = f"Workspace calibrated! Size: {width:.3f}m x {height:.3f}m"
            response.workspace.is_calibrated = True
            response.workspace.marker_ids = marker_ids

            # Add all corner positions
            for pos in corner_positions:
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = pos[2]
                response.workspace.corners.append(p)

            response.workspace.width = width
            response.workspace.height = height

            # Publish workspace
            self.workspace_pub.publish(response.workspace)

            # Publish workspace plane visualization
            self._publish_workspace_plane(corner_positions)

            # Log calibration results
            rospy.loginfo("=" * 60)
            rospy.loginfo("WORKSPACE CALIBRATED")
            rospy.loginfo("=" * 60)
            corner_names = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
            for i, (mid, pos) in enumerate(zip(marker_ids, corner_positions)):
                rospy.loginfo(f"  {corner_names[i]:12} (ID {mid:2}): ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f})")
            rospy.loginfo(f"  Workspace size: {width:.3f}m x {height:.3f}m")
            rospy.loginfo("=" * 60)

        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {str(e)}"
            response.workspace.is_calibrated = False
            rospy.logerr(f"Workspace calibration error: {e}")

        return response

    def get_target_service(self, req):
        """Service to get current target position."""
        response = GetTargetPositionResponse()

        with self.target_lock:
            if self.current_target is not None and self.current_target.valid:
                response.success = True
                response.message = "Target available"
                response.position = self.current_target.position
                response.confidence = self.current_target.confidence
            else:
                response.success = False
                response.message = "No valid target detected"
                response.position = Point()
                response.confidence = 0.0

        return response

    # ============================================================
    # SECTION 5: VISUALIZATION
    # ============================================================

    def _publish_marker_visualization(self, markers):
        """
        Publish visualization markers for RViz.
        Creates spheres at marker positions in robot frame.
        """
        viz_array = VizMarkerArray()

        for i, marker in enumerate(markers):
            if not marker.valid:
                continue

            # Transform marker position to robot frame
            if marker.position.x != 0 or marker.position.y != 0 or marker.position.z != 0:
                pos_robot = self._camera_to_robot_transform(marker.position)
                rospy.loginfo_throttle(1.0, f"Marker {marker.marker_id}: ({pos_robot[0]:.3f}, {pos_robot[1]:.3f}, {pos_robot[2]:.3f})")
            else:
                rospy.logwarn_throttle(2.0, f"Marker {marker.marker_id} has no 3D pose")
                continue

            # Create visualization marker (sphere)
            viz_marker = Marker()
            viz_marker.header.frame_id = "base_link"
            viz_marker.header.stamp = rospy.Time.now()
            viz_marker.ns = "aruco_markers"
            viz_marker.id = marker.marker_id
            viz_marker.type = Marker.SPHERE
            viz_marker.action = Marker.ADD

            # Position
            viz_marker.pose.position.x = pos_robot[0]
            viz_marker.pose.position.y = pos_robot[1]
            viz_marker.pose.position.z = pos_robot[2]
            viz_marker.pose.orientation.w = 1.0

            # Scale (5cm sphere)
            viz_marker.scale.x = 0.05
            viz_marker.scale.y = 0.05
            viz_marker.scale.z = 0.05

            # Color (blue for all markers)
            viz_marker.color.r = 0.0
            viz_marker.color.g = 0.5
            viz_marker.color.b = 1.0
            viz_marker.color.a = 0.8

            viz_marker.lifetime = rospy.Duration(0)

            # Add text label
            text_marker = Marker()
            text_marker.header = viz_marker.header
            text_marker.ns = "aruco_labels"
            text_marker.id = marker.marker_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pos_robot[0]
            text_marker.pose.position.y = pos_robot[1]
            text_marker.pose.position.z = pos_robot[2] + 0.05
            text_marker.text = f"ID: {marker.marker_id}"
            text_marker.scale.z = 0.03
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0)

            viz_array.markers.append(viz_marker)
            viz_array.markers.append(text_marker)

            # Publish TF for each marker
            self.tf_broadcaster.sendTransform(
                (pos_robot[0], pos_robot[1], pos_robot[2]),
                (0, 0, 0, 1),
                rospy.Time.now(),
                f"aruco_marker_{marker.marker_id}",
                "base_link"
            )

        # Publish visualization
        if len(viz_array.markers) > 0:
            self.marker_viz_pub.publish(viz_array)

    def _publish_workspace_plane(self, corners):
        """
        Publish workspace plane visualization.
        Creates a semi-transparent plane mesh showing the calibrated workspace.
        """
        if len(corners) != 4:
            return

        viz_array = VizMarkerArray()
        timestamp = rospy.Time.now()

        # Create plane as triangle list (2 triangles forming a rectangle)
        plane_marker = Marker()
        plane_marker.header.frame_id = "base_link"
        plane_marker.header.stamp = timestamp
        plane_marker.ns = "workspace_plane"
        plane_marker.id = 0
        plane_marker.type = Marker.TRIANGLE_LIST
        plane_marker.action = Marker.ADD

        # Semi-transparent light blue plane
        plane_marker.color.r = 0.3
        plane_marker.color.g = 0.7
        plane_marker.color.b = 1.0
        plane_marker.color.a = 0.3

        plane_marker.scale.x = 1.0
        plane_marker.scale.y = 1.0
        plane_marker.scale.z = 1.0

        # Define 2 triangles: TL-TR-BR and TL-BR-BL
        # Triangle 1: TL, TR, BR
        for idx in [0, 1, 2]:
            p = Point()
            p.x = corners[idx][0]
            p.y = corners[idx][1]
            p.z = corners[idx][2]
            plane_marker.points.append(p)

        # Triangle 2: TL, BR, BL
        for idx in [0, 2, 3]:
            p = Point()
            p.x = corners[idx][0]
            p.y = corners[idx][1]
            p.z = corners[idx][2]
            plane_marker.points.append(p)

        viz_array.markers.append(plane_marker)

        # Create border lines around workspace
        border_marker = Marker()
        border_marker.header.frame_id = "base_link"
        border_marker.header.stamp = timestamp
        border_marker.ns = "workspace_border"
        border_marker.id = 1
        border_marker.type = Marker.LINE_STRIP
        border_marker.action = Marker.ADD

        # Bright blue border
        border_marker.color.r = 0.0
        border_marker.color.g = 0.5
        border_marker.color.b = 1.0
        border_marker.color.a = 1.0

        border_marker.scale.x = 0.005  # 5mm thick line

        # Add all 4 corners + first corner again to close the loop
        for idx in [0, 1, 2, 3, 0]:
            p = Point()
            p.x = corners[idx][0]
            p.y = corners[idx][1]
            p.z = corners[idx][2]
            border_marker.points.append(p)

        viz_array.markers.append(border_marker)

        # Publish workspace plane
        self.workspace_plane_pub.publish(viz_array)
        rospy.loginfo("Published workspace plane visualization")


# ============================================================
# MAIN
# ============================================================

def main():
    try:
        node = WorkspaceDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
