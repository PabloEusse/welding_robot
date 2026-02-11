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

from trajectory_tracker.msg import TrajectoryTarget
from trajectory_tracker.srv import GetTargetPosition, GetTargetPositionResponse


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

        # Publishers
        self.target_pub = rospy.Publisher(
            '/trajectory_tracker/target', TrajectoryTarget, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber(
            '/marker_detector/detection', TrajectoryTarget, self.detection_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            '/camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        # Services
        self.get_target_srv = rospy.Service(
            '/trajectory_tracker/get_target', GetTargetPosition, self.get_target_service)

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


def main():
    try:
        node = CoordinateTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
