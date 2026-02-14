#!/usr/bin/env python3
"""
Line Extractor Node
===================

Extracts drawn lines from camera images and converts them to robot trajectories.

This node:
1. Captures current camera image
2. Detects colored lines using HSV filtering
3. Extracts line skeleton and orders points
4. Applies smoothing to reduce pixel-level jitter
5. Transforms points to robot frame using workspace calibration
6. Publishes as Path for visualization and recording

Services:
    /trajectory_tracker/extract_line (ExtractLine): Extract line from current image

Topics Published:
    /trajectory_tracker/extracted_path (nav_msgs/Path): Extracted line as path
    /trajectory_tracker/extraction_debug (sensor_msgs/Image): Debug visualization
"""

import rospy
import cv2
import numpy as np
from threading import Lock
from scipy import interpolate
from scipy.signal import savgol_filter

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

from trajectory_tracker.msg import TrajectoryTarget, WorkspaceCalibration
from trajectory_tracker.srv import ExtractLine, ExtractLineResponse

# OpenCV version compatibility
CV_VERSION = cv2.__version__.split('.')
CV_MAJOR = int(CV_VERSION[0])
CV_MINOR = int(CV_VERSION[1]) if len(CV_VERSION) > 1 else 0

rospy.loginfo(f"OpenCV version: {cv2.__version__}")


class LineExtractorNode:
    def __init__(self):
        rospy.init_node('line_extractor_node')

        # Shutdown flag
        self.is_shutdown = False

        # Parameters
        self.color_lower1 = np.array(rospy.get_param('~color_lower1', [100, 100, 100]))
        self.color_upper1 = np.array(rospy.get_param('~color_upper1', [130, 255, 255]))
        self.color_lower2 = np.array(rospy.get_param('~color_lower2', [100, 100, 100]))
        self.color_upper2 = np.array(rospy.get_param('~color_upper2', [130, 255, 255]))
        self.min_line_length = rospy.get_param('~min_line_length', 50)  # pixels
        self.sampling_distance = rospy.get_param('~sampling_distance', 5)  # pixels between points

        # Smoothing parameters
        self.enable_smoothing = rospy.get_param('~enable_smoothing', True)
        self.smoothing_method = rospy.get_param('~smoothing_method', 'bspline')  # 'bspline', 'savgol', 'gaussian'
        self.bspline_smoothing = rospy.get_param('~bspline_smoothing', 5.0)  # smoothing factor for B-spline
        self.savgol_window = rospy.get_param('~savgol_window', 11)  # window length for Savitzky-Golay
        self.savgol_order = rospy.get_param('~savgol_order', 3)  # polynomial order for Savitzky-Golay
        self.gaussian_kernel = rospy.get_param('~gaussian_kernel', 5)  # kernel size for Gaussian smoothing

        # CV Bridge
        self.bridge = CvBridge()

        # State
        self.current_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.workspace = None
        self.image_lock = Lock()

        # Coordinate transformation parameters
        self.plane_z = rospy.get_param('/coordinate_transformer_node/plane_z', 0.0)
        cam_pose = rospy.get_param('/coordinate_transformer_node/camera_to_robot',
                                    [0.5, 0.0, 0.8, 3.14159, 0.0, 0.0])
        self.T_robot_camera = self._pose_to_matrix(cam_pose)

        # Publishers
        self.path_pub = rospy.Publisher(
            '/trajectory_tracker/extracted_path', Path, queue_size=1)
        self.debug_pub = rospy.Publisher(
            '/trajectory_tracker/extraction_debug', Image, queue_size=1)
        self.contour_marker_pub = rospy.Publisher(
            '/trajectory_tracker/line_contour_markers', MarkerArray, queue_size=1)
        self.skeleton_marker_pub = rospy.Publisher(
            '/trajectory_tracker/line_skeleton_markers', MarkerArray, queue_size=1)

        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            '/camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        self.workspace_sub = rospy.Subscriber(
            '/trajectory_tracker/workspace', WorkspaceCalibration, self.workspace_callback, queue_size=1)

        # Service
        self.extract_srv = rospy.Service(
            '/trajectory_tracker/extract_line', ExtractLine, self.extract_line_service)

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Line Extractor Node started")
        rospy.loginfo("Call /trajectory_tracker/extract_line to extract drawn line")

    def shutdown(self):
        """Clean up resources on shutdown."""
        rospy.loginfo("Shutting down line extractor node...")
        self.is_shutdown = True
        rospy.loginfo("Line extractor node shutdown complete")

    def _pose_to_matrix(self, pose):
        """Convert [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix."""
        import tf.transformations as tft
        x, y, z, roll, pitch, yaw = pose
        T = tft.euler_matrix(roll, pitch, yaw)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def image_callback(self, msg):
        """Store latest image."""
        with self.image_lock:
            try:
                self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                rospy.logwarn(f"Failed to convert image: {e}")

    def camera_info_callback(self, msg):
        """Store camera calibration."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera calibration received")

    def workspace_callback(self, msg):
        """Store workspace calibration."""
        self.workspace = msg
        if msg.is_calibrated:
            rospy.loginfo("Workspace calibration received")

    def extract_line_service(self, req):
        """Service to extract line from current image."""
        response = ExtractLineResponse()

        with self.image_lock:
            if self.current_image is None:
                response.success = False
                response.message = "No camera image available!"
                response.num_points = 0
                return response

            image = self.current_image.copy()

        if self.camera_matrix is None:
            response.success = False
            response.message = "Camera not calibrated!"
            response.num_points = 0
            return response

        rospy.loginfo("=" * 60)
        rospy.loginfo("EXTRACTING LINE FROM IMAGE")
        rospy.loginfo("=" * 60)

        # Extract line from image
        line_points, debug_image, mask = self.extract_line_from_image(image)

        if len(line_points) == 0:
            response.success = False
            response.message = "No line detected! Make sure the line is visible and in the correct color."
            response.num_points = 0

            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                self.debug_pub.publish(debug_msg)
            except:
                pass

            return response

        rospy.loginfo(f"Detected {len(line_points)} points on the line")

        # Transform points to robot frame
        path_points = []
        for px, py in line_points:
            try:
                pos_robot = self.pixel_to_robot(px, py)
                path_points.append(pos_robot)
            except Exception as e:
                rospy.logwarn(f"Failed to transform point ({px}, {py}): {e}")
                continue

        if len(path_points) == 0:
            response.success = False
            response.message = "Failed to transform line points to robot frame!"
            response.num_points = 0
            return response

        # Create and publish Path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "base_link"

        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # Publish visualization markers for RViz
        self._publish_visualization_markers(mask, line_points, path_points)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except:
            pass

        response.success = True
        response.message = f"Successfully extracted {len(path_points)} points!"
        response.num_points = len(path_points)

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"LINE EXTRACTION COMPLETE")
        rospy.loginfo(f"  Points extracted: {len(path_points)}")
        rospy.loginfo(f"  Path published to: /trajectory_tracker/extracted_path")
        rospy.loginfo("=" * 60)

        return response

    def extract_line_from_image(self, image):
        """
        Extract line skeleton from image.
        Returns: (list of (x,y) points, debug_image, mask)
        """
        debug_image = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for colored line
        mask1 = cv2.inRange(hsv, self.color_lower1, self.color_upper1)
        mask2 = cv2.inRange(hsv, self.color_lower2, self.color_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # Thin the line to skeleton
        skeleton = self.skeletonize(mask)

        # Find ordered points along the skeleton
        points = self.order_skeleton_points(skeleton)

        # Downsample points
        if len(points) > 0:
            points = self.downsample_points(points, self.sampling_distance)

        # Apply smoothing
        if len(points) > 0 and self.enable_smoothing:
            points = self.smooth_points(points)

        # Draw on debug image
        if len(points) > 0:
            # Draw original mask
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            debug_image = cv2.addWeighted(debug_image, 0.7, mask_bgr, 0.3, 0)

            # Draw skeleton
            for i, (x, y) in enumerate(points):
                cv2.circle(debug_image, (int(x), int(y)), 3, (0, 255, 0), -1)
                if i > 0:
                    prev_x, prev_y = points[i-1]
                    cv2.line(debug_image, (int(prev_x), int(prev_y)),
                            (int(x), int(y)), (255, 0, 0), 2)

            # Draw start and end
            cv2.circle(debug_image, (int(points[0][0]), int(points[0][1])),
                      8, (0, 255, 255), -1)  # Yellow start
            cv2.circle(debug_image, (int(points[-1][0]), int(points[-1][1])),
                      8, (255, 0, 255), -1)  # Magenta end

            cv2.putText(debug_image, f"{len(points)} points", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(debug_image, "No line detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return points, debug_image, mask

    def skeletonize(self, mask):
        """Extract skeleton of binary mask."""
        # Simple thinning algorithm
        skeleton = np.zeros_like(mask)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

        temp = mask.copy()
        while True:
            eroded = cv2.erode(temp, element)
            opened = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, element)
            subset = eroded - opened
            skeleton = cv2.bitwise_or(skeleton, subset)
            temp = eroded.copy()

            if cv2.countNonZero(temp) == 0:
                break

        return skeleton

    def order_skeleton_points(self, skeleton):
        """
        Order skeleton points from one end to the other.
        Uses a simple traversal algorithm.
        """
        # Find all skeleton points
        points = np.column_stack(np.where(skeleton > 0))

        if len(points) < self.min_line_length:
            return []

        # Convert to (x, y) format
        points = [(p[1], p[0]) for p in points]  # (row, col) -> (x, y)

        # Find endpoints (points with only one neighbor)
        endpoints = self.find_endpoints(skeleton)

        if len(endpoints) < 2:
            # If no clear endpoints, use leftmost and rightmost points
            points_arr = np.array(points)
            start_idx = np.argmin(points_arr[:, 0])
            start = points[start_idx]
        else:
            start = (endpoints[0][1], endpoints[0][0])

        # Order points using nearest neighbor traversal
        ordered = [start]
        remaining = set(points)
        remaining.remove(start)

        current = start
        while remaining:
            # Find nearest unvisited point
            min_dist = float('inf')
            nearest = None

            for point in remaining:
                dist = np.sqrt((point[0] - current[0])**2 + (point[1] - current[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest = point

            if nearest is None or min_dist > 20:  # Break if gap is too large
                break

            ordered.append(nearest)
            remaining.remove(nearest)
            current = nearest

        return ordered

    def find_endpoints(self, skeleton):
        """Find endpoints of skeleton (pixels with only 1 neighbor)."""
        endpoints = []
        points = np.column_stack(np.where(skeleton > 0))

        for r, c in points:
            # Count neighbors
            neighbors = 0
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < skeleton.shape[0] and 0 <= nc < skeleton.shape[1]:
                        if skeleton[nr, nc] > 0:
                            neighbors += 1

            if neighbors == 1:
                endpoints.append((r, c))

        return endpoints

    def downsample_points(self, points, distance):
        """Downsample points to maintain minimum distance between consecutive points."""
        if len(points) == 0:
            return []

        downsampled = [points[0]]
        last_point = points[0]

        for point in points[1:]:
            dist = np.sqrt((point[0] - last_point[0])**2 + (point[1] - last_point[1])**2)
            if dist >= distance:
                downsampled.append(point)
                last_point = point

        return downsampled

    def smooth_points(self, points):
        """
        Smooth a list of (x, y) points to reduce pixel-level jitter.

        Methods:
        - 'bspline': B-spline interpolation (smooth curves)
        - 'savgol': Savitzky-Golay filter (preserves features)
        - 'gaussian': Gaussian smoothing (simple averaging)

        Args:
            points: List of (x, y) tuples

        Returns:
            List of smoothed (x, y) tuples
        """
        if len(points) < 4:
            return points  # Not enough points to smooth

        points_array = np.array(points)
        x = points_array[:, 0]
        y = points_array[:, 1]

        try:
            if self.smoothing_method == 'bspline':
                # B-spline interpolation for smooth curves
                num_points = len(points)

                # Parameterize by cumulative chord length
                t = np.zeros(num_points)
                for i in range(1, num_points):
                    t[i] = t[i-1] + np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)

                if t[-1] == 0:  # All points are the same
                    return points

                # Normalize t to [0, 1]
                t = t / t[-1]

                # Determine spline degree (k) based on number of points
                k = min(3, num_points - 1)  # Cubic spline if possible

                # Create B-spline with smoothing
                tck_x, _ = interpolate.splprep([x], u=t, s=self.bspline_smoothing, k=k)
                tck_y, _ = interpolate.splprep([y], u=t, s=self.bspline_smoothing, k=k)

                # Evaluate spline at original parameter values
                x_smooth = interpolate.splev(t, tck_x)[0]
                y_smooth = interpolate.splev(t, tck_y)[0]

            elif self.smoothing_method == 'savgol':
                # Savitzky-Golay filter (preserves peak shapes)
                window = min(self.savgol_window, len(points))
                if window % 2 == 0:  # Must be odd
                    window += 1

                order = min(self.savgol_order, window - 1)

                x_smooth = savgol_filter(x, window, order, mode='nearest')
                y_smooth = savgol_filter(y, window, order, mode='nearest')

            elif self.smoothing_method == 'gaussian':
                # Gaussian smoothing (simple averaging)
                kernel_size = self.gaussian_kernel
                if kernel_size % 2 == 0:  # Must be odd
                    kernel_size += 1

                # Create Gaussian kernel
                sigma = kernel_size / 3.0
                kernel = cv2.getGaussianKernel(kernel_size, sigma)
                kernel = kernel / kernel.sum()

                # Apply 1D convolution
                x_smooth = np.convolve(x, kernel.flatten(), mode='same')
                y_smooth = np.convolve(y, kernel.flatten(), mode='same')

            else:
                rospy.logwarn(f"Unknown smoothing method: {self.smoothing_method}, using original points")
                return points

            # Convert back to list of tuples
            smoothed_points = [(x_smooth[i], y_smooth[i]) for i in range(len(x_smooth))]

            rospy.loginfo(f"Applied {self.smoothing_method} smoothing to {len(points)} points")
            return smoothed_points

        except Exception as e:
            rospy.logwarn(f"Smoothing failed: {e}, using original points")
            return points

    def pixel_to_robot(self, u, v):
        """Transform pixel coordinates to robot frame."""
        # Unproject pixel to 3D ray in camera frame
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Ray direction in camera frame (normalized)
        x_cam = (u - cx) / fx
        y_cam = (v - cy) / fy
        z_cam = 1.0

        # Scale ray to intersect with plane at z = plane_z in robot frame
        # Transform ray to robot frame and solve for intersection

        # Camera origin in robot frame
        cam_origin = self.T_robot_camera[:3, 3]

        # Ray direction in robot frame
        ray_cam = np.array([x_cam, y_cam, z_cam, 0])
        ray_robot = self.T_robot_camera @ ray_cam
        ray_dir = ray_robot[:3]

        # Intersect ray with plane z = plane_z
        # cam_origin + t * ray_dir has z = plane_z
        # cam_origin[2] + t * ray_dir[2] = plane_z
        if abs(ray_dir[2]) < 1e-6:
            raise ValueError("Ray parallel to plane")

        t = (self.plane_z - cam_origin[2]) / ray_dir[2]

        if t < 0:
            raise ValueError("Intersection behind camera")

        point_robot = cam_origin + t * ray_dir

        return point_robot

    def _publish_visualization_markers(self, mask, pixel_points, robot_points):
        """
        Publish visualization markers for RViz.

        Creates two types of markers:
        1. Line contour (boundary of detected line) - RED
        2. Line skeleton (centerline/trajectory) - GREEN

        Args:
            mask: Binary mask of detected line
            pixel_points: List of (x,y) pixel coordinates of skeleton
            robot_points: List of 3D points in robot frame
        """
        timestamp = rospy.Time.now()

        # === 1. Contour Markers (boundary of original line) ===
        contour_markers = MarkerArray()

        # Find contours from binary mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i, contour in enumerate(contours):
            if len(contour) < 3:
                continue

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = timestamp
            marker.ns = "line_contour"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Red color for contour boundary
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            # Line width (2mm)
            marker.scale.x = 0.002

            # Transform contour pixel coordinates to robot frame
            for point_px in contour:
                px, py = point_px[0]
                try:
                    pos_robot = self.pixel_to_robot(float(px), float(py))
                    p = Point()
                    p.x = pos_robot[0]
                    p.y = pos_robot[1]
                    p.z = pos_robot[2]
                    marker.points.append(p)
                except:
                    continue

            if len(marker.points) > 0:
                contour_markers.markers.append(marker)

        self.contour_marker_pub.publish(contour_markers)

        # === 2. Skeleton Markers (centerline/trajectory) ===
        skeleton_markers = MarkerArray()

        if len(robot_points) > 0:
            # Create line strip for trajectory centerline
            line_marker = Marker()
            line_marker.header.frame_id = "base_link"
            line_marker.header.stamp = timestamp
            line_marker.ns = "line_skeleton"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            # Green color for skeleton trajectory
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            # Line width (5mm)
            line_marker.scale.x = 0.005

            # Add all trajectory points
            for pos_robot in robot_points:
                p = Point()
                p.x = pos_robot[0]
                p.y = pos_robot[1]
                p.z = pos_robot[2]
                line_marker.points.append(p)

            skeleton_markers.markers.append(line_marker)

            # Create sphere marker for start point (Yellow)
            start_marker = Marker()
            start_marker.header.frame_id = "base_link"
            start_marker.header.stamp = timestamp
            start_marker.ns = "line_skeleton_points"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = robot_points[0][0]
            start_marker.pose.position.y = robot_points[0][1]
            start_marker.pose.position.z = robot_points[0][2]
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.01
            start_marker.color.r = 1.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            skeleton_markers.markers.append(start_marker)

            # Create sphere marker for end point (Magenta)
            end_marker = Marker()
            end_marker.header.frame_id = "base_link"
            end_marker.header.stamp = timestamp
            end_marker.ns = "line_skeleton_points"
            end_marker.id = 2
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position.x = robot_points[-1][0]
            end_marker.pose.position.y = robot_points[-1][1]
            end_marker.pose.position.z = robot_points[-1][2]
            end_marker.pose.orientation.w = 1.0
            end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = 0.01
            end_marker.color.r = 1.0
            end_marker.color.g = 0.0
            end_marker.color.b = 1.0
            end_marker.color.a = 1.0
            skeleton_markers.markers.append(end_marker)

        self.skeleton_marker_pub.publish(skeleton_markers)


def main():
    try:
        node = LineExtractorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
