#!/usr/bin/env python3
"""
Coordinate Transformer Node
============================

Transforms detected pixel coordinates to 3D positions in the robot base frame.

Calibration Methods:
1. FIXED_PLANE: Assume drawing surface is at fixed Z height (simple, requires manual setup)
2. MARKER_POSE: Use ArUco marker pose estimation (more accurate, requires marker)
3. HOMOGRAPHY: Use 4-point calibration homography (good for flat surfaces)

The transformation chain:
    pixel (u,v) → camera frame (x,y,z) → robot base frame (X,Y,Z)

Topics Subscribed:
    /marker_detector/detection (TrajectoryTarget): Detected marker in pixel coords
    /camera/camera_info (CameraInfo): Camera calibration

Topics Published:
    /trajectory_tracker/target (TrajectoryTarget): Target in robot base frame

Services:
    /trajectory_tracker/get_target (GetTargetPosition): Get current target position

Parameters:
    ~transform_mode (str): "fixed_plane", "marker_pose", or "homography"
    ~plane_z (float): Z height of drawing plane in robot frame (for fixed_plane mode)
    ~camera_to_robot (list): [x, y, z, roll, pitch, yaw] camera pose in robot frame
"""

import rospy
import numpy as np
import tf
import tf.transformations as tft
from threading import Lock

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray as VizMarkerArray

from trajectory_tracker.msg import TrajectoryTarget, MarkerArray, WorkspaceCalibration
from trajectory_tracker.srv import GetTargetPosition, GetTargetPositionResponse, CalibrateWorkspace, CalibrateWorkspaceResponse


class CoordinateTransformerNode:
    def __init__(self):
        rospy.init_node('coordinate_transformer_node')

        # Parameters
        self.transform_mode = rospy.get_param('~transform_mode', 'fixed_plane')
        self.plane_z = rospy.get_param('~plane_z', 0.0)  # Drawing surface Z in robot frame

        # Camera to robot base transformation
        # Default: camera looking down at workspace
        # [x, y, z, roll, pitch, yaw] in robot base frame
        cam_pose = rospy.get_param('~camera_to_robot', [0.5, 0.0, 0.8, 3.14159, 0.0, 0.0])
        self.T_robot_camera = self._pose_to_matrix(cam_pose)

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_matrix_inv = None

        # Homography matrix (for homography mode)
        # This should be calibrated using 4 known points
        self.homography = None

        # Current detection storage
        self.current_target = None
        self.target_lock = Lock()

        # Workspace calibration (using 2 markers for 2-point calibration)
        self.workspace_calibrated = False
        self.workspace_marker_ids = [0, 1]  # Default marker IDs
        self.workspace_corner_0 = None  # Robot frame position of marker 0
        self.workspace_corner_1 = None  # Robot frame position of marker 1
        self.workspace_markers = {}  # Store detected markers {id: TrajectoryTarget}

        # Publishers
        self.target_pub = rospy.Publisher(
            '/trajectory_tracker/target', TrajectoryTarget, queue_size=1)
        self.workspace_pub = rospy.Publisher(
            '/trajectory_tracker/workspace', WorkspaceCalibration, queue_size=1)
        self.marker_viz_pub = rospy.Publisher(
            '/trajectory_tracker/marker_visualization', VizMarkerArray, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber(
            '/marker_detector/detection', TrajectoryTarget, self.detection_callback, queue_size=1)
        self.marker_array_sub = rospy.Subscriber(
            '/marker_detector/markers', MarkerArray, self.marker_array_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            '/camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        # Services
        self.get_target_srv = rospy.Service(
            '/trajectory_tracker/get_target', GetTargetPosition, self.get_target_service)
        self.calibrate_workspace_srv = rospy.Service(
            '/trajectory_tracker/calibrate_workspace', CalibrateWorkspace, self.calibrate_workspace_service)

        # TF broadcaster for visualization
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo(f"Coordinate transformer started in {self.transform_mode.upper()} mode")
        rospy.loginfo(f"Drawing plane Z: {self.plane_z} m")

    def _pose_to_matrix(self, pose):
        """Convert [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix."""
        x, y, z, roll, pitch, yaw = pose
        T = tft.euler_matrix(roll, pitch, yaw)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration data."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
            rospy.loginfo("Camera calibration received")

    def detection_callback(self, msg: TrajectoryTarget):
        """
        Transform detected pixel coordinates to robot base frame.
        """
        if self.camera_matrix is None:
            rospy.logwarn_throttle(2.0, "Waiting for camera calibration...")
            return

        if not msg.valid:
            return

        # Transform based on mode
        if self.transform_mode == 'fixed_plane':
            position = self.pixel_to_robot_fixed_plane(msg.pixel_x, msg.pixel_y)
        elif self.transform_mode == 'marker_pose':
            position = self.marker_pose_to_robot(msg)
        elif self.transform_mode == 'homography':
            position = self.pixel_to_robot_homography(msg.pixel_x, msg.pixel_y)
        else:
            rospy.logwarn_throttle(5.0, f"Unknown transform mode: {self.transform_mode}")
            return

        if position is None:
            return

        # Create output message
        target = TrajectoryTarget()
        target.header.stamp = msg.header.stamp
        target.header.frame_id = "base_link"  # Robot base frame
        target.position.x = position[0]
        target.position.y = position[1]
        target.position.z = position[2]
        target.pixel_x = msg.pixel_x
        target.pixel_y = msg.pixel_y
        target.confidence = msg.confidence
        target.marker_id = msg.marker_id
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

    def pixel_to_robot_fixed_plane(self, u, v):
        """
        ================================================================
        FIXED PLANE METHOD
        ================================================================
        Assumes the drawing surface is at a fixed Z height in robot frame.

        Steps:
        1. Pixel to normalized camera coords: (u,v) → (x_n, y_n, 1)
           [x_n]         [u]
           [y_n] = K^-1 * [v]
           [1  ]         [1]

        2. Ray direction in camera frame: d_cam = [x_n, y_n, 1]^T

        3. Transform ray to robot frame using camera pose

        4. Intersect ray with plane Z = plane_z:
           P = P_cam + t * d_robot
           where t = (plane_z - P_cam.z) / d_robot.z

        ================================================================
        """
        if self.camera_matrix_inv is None:
            return None

        # Step 1: Pixel to normalized coordinates
        pixel_homogeneous = np.array([u, v, 1.0])
        ray_camera = self.camera_matrix_inv @ pixel_homogeneous
        ray_camera = ray_camera / np.linalg.norm(ray_camera)  # Normalize

        # Step 2: Transform ray direction to robot frame
        # R_robot_camera is the rotation part of T_robot_camera
        R_robot_camera = self.T_robot_camera[:3, :3]
        ray_robot = R_robot_camera @ ray_camera

        # Camera origin in robot frame
        cam_origin_robot = self.T_robot_camera[:3, 3]

        # Step 3: Intersect ray with plane Z = plane_z
        # Ray: P = cam_origin + t * ray_robot
        # Plane: P.z = plane_z
        # Solve: cam_origin.z + t * ray_robot.z = plane_z
        if abs(ray_robot[2]) < 1e-6:
            rospy.logwarn_throttle(1.0, "Ray parallel to plane, cannot compute intersection")
            return None

        t = (self.plane_z - cam_origin_robot[2]) / ray_robot[2]

        if t < 0:
            rospy.logwarn_throttle(1.0, "Intersection behind camera")
            return None

        # Compute intersection point
        intersection = cam_origin_robot + t * ray_robot

        return intersection

    def marker_pose_to_robot(self, msg: TrajectoryTarget):
        """
        ================================================================
        MARKER POSE METHOD
        ================================================================
        Uses the 3D pose from ArUco marker detection (already in camera frame)
        and transforms to robot frame.

        The marker detector node provides position in camera frame when
        ArUco pose estimation is used.
        ================================================================
        """
        # Position from marker detection (in camera frame)
        p_camera = np.array([msg.position.x, msg.position.y, msg.position.z, 1.0])

        # Transform to robot frame
        p_robot = self.T_robot_camera @ p_camera

        return p_robot[:3]

    def pixel_to_robot_homography(self, u, v):
        """
        ================================================================
        HOMOGRAPHY METHOD
        ================================================================
        Uses a pre-calibrated homography matrix to map pixel coordinates
        directly to robot XY coordinates on the drawing plane.

        Homography: [X]     [u]
                    [Y] = H [v]
                    [1]     [1]

        This requires calibration with 4+ known points.
        ================================================================
        """
        if self.homography is None:
            rospy.logwarn_throttle(5.0, "Homography not calibrated! Using fixed_plane instead.")
            return self.pixel_to_robot_fixed_plane(u, v)

        # Apply homography
        pixel_homogeneous = np.array([u, v, 1.0])
        robot_homogeneous = self.homography @ pixel_homogeneous
        robot_homogeneous = robot_homogeneous / robot_homogeneous[2]

        return np.array([robot_homogeneous[0], robot_homogeneous[1], self.plane_z])

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

    def marker_array_callback(self, msg: MarkerArray):
        """
        Store all detected markers for workspace calibration and visualize in RViz.
        """
        # Store markers by ID
        for marker in msg.markers:
            if marker.valid:
                self.workspace_markers[marker.marker_id] = marker

        # Visualize all markers in RViz
        if len(msg.markers) > 0:
            self._publish_marker_visualization(msg.markers)

    def calibrate_workspace_service(self, req):
        """
        Calibrate workspace using two markers.

        This service:
        1. Detects two markers (specified by marker_id_0 and marker_id_1)
        2. Transforms their positions to robot frame
        3. Defines workspace coordinate system based on these two points

        The two markers should be placed at opposite corners of the workspace.
        """
        response = CalibrateWorkspaceResponse()
        response.workspace = WorkspaceCalibration()
        response.workspace.header.stamp = rospy.Time.now()
        response.workspace.header.frame_id = "robot_base"

        # Use requested marker IDs or defaults
        marker_id_0 = req.marker_id_0 if req.marker_id_0 >= 0 else 0
        marker_id_1 = req.marker_id_1 if req.marker_id_1 >= 0 else 1

        rospy.loginfo(f"Calibrating workspace using markers {marker_id_0} and {marker_id_1}...")

        # Check if both markers are detected
        if marker_id_0 not in self.workspace_markers:
            response.success = False
            response.message = f"Marker ID {marker_id_0} not detected! Place it in camera view."
            response.workspace.is_calibrated = False
            return response

        if marker_id_1 not in self.workspace_markers:
            response.success = False
            response.message = f"Marker ID {marker_id_1} not detected! Place it in camera view."
            response.workspace.is_calibrated = False
            return response

        # Get markers
        marker_0 = self.workspace_markers[marker_id_0]
        marker_1 = self.workspace_markers[marker_id_1]

        # Transform marker positions to robot frame
        try:
            # For marker_pose mode, use the 3D position from ArUco pose estimation
            if self.transform_mode == 'marker_pose' and marker_0.position.x != 0:
                # Markers already have 3D positions in camera frame
                pos_0_robot = self.camera_to_robot_transform(marker_0.position)
                pos_1_robot = self.camera_to_robot_transform(marker_1.position)
            else:
                # Use pixel coordinates with fixed plane assumption
                pos_0_robot = self.pixel_to_robot_fixed_plane(marker_0.pixel_x, marker_0.pixel_y)
                pos_1_robot = self.pixel_to_robot_fixed_plane(marker_1.pixel_x, marker_1.pixel_y)

            self.workspace_corner_0 = pos_0_robot
            self.workspace_corner_1 = pos_1_robot
            self.workspace_marker_ids = [marker_id_0, marker_id_1]
            self.workspace_calibrated = True

            # Calculate workspace dimensions
            width = abs(pos_1_robot[0] - pos_0_robot[0])
            height = abs(pos_1_robot[1] - pos_0_robot[1])

            # Fill response
            response.success = True
            response.message = f"Workspace calibrated! Size: {width:.3f}m x {height:.3f}m"
            response.workspace.is_calibrated = True
            response.workspace.marker_id_0 = marker_id_0
            response.workspace.marker_id_1 = marker_id_1
            response.workspace.corner_0.x = pos_0_robot[0]
            response.workspace.corner_0.y = pos_0_robot[1]
            response.workspace.corner_0.z = pos_0_robot[2]
            response.workspace.corner_1.x = pos_1_robot[0]
            response.workspace.corner_1.y = pos_1_robot[1]
            response.workspace.corner_1.z = pos_1_robot[2]
            response.workspace.width = width
            response.workspace.height = height

            # Publish workspace
            self.workspace_pub.publish(response.workspace)

            rospy.loginfo(f"Workspace calibrated:")
            rospy.loginfo(f"  Marker {marker_id_0} at: ({pos_0_robot[0]:.3f}, {pos_0_robot[1]:.3f}, {pos_0_robot[2]:.3f})")
            rospy.loginfo(f"  Marker {marker_id_1} at: ({pos_1_robot[0]:.3f}, {pos_1_robot[1]:.3f}, {pos_1_robot[2]:.3f})")
            rospy.loginfo(f"  Workspace size: {width:.3f}m x {height:.3f}m")

        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {str(e)}"
            response.workspace.is_calibrated = False
            rospy.logerr(f"Workspace calibration error: {e}")

        return response

    def camera_to_robot_transform(self, point_camera):
        """Transform a point from camera frame to robot frame."""
        p_camera = np.array([point_camera.x, point_camera.y, point_camera.z, 1.0])
        p_robot = self.T_robot_camera @ p_camera
        return p_robot[:3]

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
                # Marker has 3D pose from ArUco
                pos_robot = self.camera_to_robot_transform(marker.position)
                rospy.loginfo_throttle(1.0, f"Marker {marker.marker_id} (ArUco pose): ({pos_robot[0]:.3f}, {pos_robot[1]:.3f}, {pos_robot[2]:.3f})")
            else:
                # Use pixel coordinates with fixed plane
                try:
                    pos_robot = self.pixel_to_robot_fixed_plane(marker.pixel_x, marker.pixel_y)
                    rospy.loginfo_throttle(1.0, f"Marker {marker.marker_id} (pixel {marker.pixel_x:.0f}, {marker.pixel_y:.0f}): ({pos_robot[0]:.3f}, {pos_robot[1]:.3f}, {pos_robot[2]:.3f})")
                except Exception as e:
                    rospy.logwarn_throttle(2.0, f"Failed to transform marker {marker.marker_id}: {e}")
                    continue

            # Create visualization marker
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

            # Color based on marker ID
            if marker.marker_id == 0:
                # Red for marker 0
                viz_marker.color.r = 1.0
                viz_marker.color.g = 0.0
                viz_marker.color.b = 0.0
            elif marker.marker_id == 1:
                # Green for marker 1
                viz_marker.color.r = 0.0
                viz_marker.color.g = 1.0
                viz_marker.color.b = 0.0
            else:
                # Blue for other markers
                viz_marker.color.r = 0.0
                viz_marker.color.g = 0.0
                viz_marker.color.b = 1.0
            viz_marker.color.a = 0.8

            viz_marker.lifetime = rospy.Duration(0)  # Never expire (until deleted)

            # Add text label
            text_marker = Marker()
            text_marker.header = viz_marker.header
            text_marker.ns = "aruco_labels"
            text_marker.id = marker.marker_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pos_robot[0]
            text_marker.pose.position.y = pos_robot[1]
            text_marker.pose.position.z = pos_robot[2] + 0.05  # Above the sphere
            text_marker.text = f"ID: {marker.marker_id}"
            text_marker.scale.z = 0.03  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0)  # Never expire

            viz_array.markers.append(viz_marker)
            viz_array.markers.append(text_marker)

            # Also publish TF for each marker
            self.tf_broadcaster.sendTransform(
                (pos_robot[0], pos_robot[1], pos_robot[2]),
                (0, 0, 0, 1),  # quaternion (no rotation)
                rospy.Time.now(),
                f"aruco_marker_{marker.marker_id}",
                "base_link"
            )

        # Publish visualization
        if len(viz_array.markers) > 0:
            rospy.loginfo_throttle(1.0, f"Publishing {len(viz_array.markers)} visualization markers (spheres + labels)")
            self.marker_viz_pub.publish(viz_array)
        else:
            rospy.logwarn_throttle(2.0, "No markers to visualize!")


def main():
    try:
        node = CoordinateTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
