#!/usr/bin/env python3
"""
Line Extractor Node
===================

Extracts drawn lines from camera images and converts them to robot trajectories.

This node:
1. Captures current camera image
2. Detects colored lines using HSV filtering
3. Extracts line skeleton and orders points
4. Transforms points to robot frame using workspace calibration
5. Publishes as Path for visualization and recording

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

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from trajectory_tracker.msg import TrajectoryTarget, WorkspaceCalibration
from trajectory_tracker.srv import ExtractLine, ExtractLineResponse


class LineExtractorNode:
    def __init__(self):
        rospy.init_node('line_extractor_node')

        # Parameters
        self.color_lower1 = np.array(rospy.get_param('~color_lower1', [100, 100, 100]))
        self.color_upper1 = np.array(rospy.get_param('~color_upper1', [130, 255, 255]))
        self.color_lower2 = np.array(rospy.get_param('~color_lower2', [100, 100, 100]))
        self.color_upper2 = np.array(rospy.get_param('~color_upper2', [130, 255, 255]))
        self.min_line_length = rospy.get_param('~min_line_length', 50)  # pixels
        self.sampling_distance = rospy.get_param('~sampling_distance', 5)  # pixels between points

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

        rospy.loginfo("Line Extractor Node started")
        rospy.loginfo("Call /trajectory_tracker/extract_line to extract drawn line")

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
        line_points, debug_image = self.extract_line_from_image(image)

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
        Returns: (list of (x,y) points, debug_image)
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

        return points, debug_image

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


def main():
    try:
        node = LineExtractorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
