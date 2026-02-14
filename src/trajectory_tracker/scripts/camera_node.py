#!/usr/bin/env python3
"""
Camera Node for DroidCam
========================

Connects to DroidCam (or any V4L2 camera) and publishes images to ROS.

DroidCam Setup:
1. Install DroidCam app on phone
2. Install DroidCam client on PC: sudo apt install droidcam
3. Connect phone via USB or WiFi
4. Run: droidcam-cli <phone_ip> 4747
5. Camera appears as /dev/video0

Topics Published:
    /camera/image_raw (sensor_msgs/Image): Raw camera image
    /camera/image_compressed (sensor_msgs/CompressedImage): JPEG compressed image
    /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info

Parameters:
    ~device_id (int): Camera device ID (default: 0)
    ~width (int): Image width (default: 1280)
    ~height (int): Image height (default: 720)
    ~fps (int): Target FPS (default: 30)
    ~show_gui (bool): Show preview window (default: False)
    ~calibration_file (str): Path to camera calibration YAML
"""

import rospy
import cv2
import numpy as np
import yaml
from pathlib import Path

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge


class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')

        # Parameters
        self.device_id = rospy.get_param('~device_id', 0)
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.show_gui = rospy.get_param('~show_gui', False)
        self.calibration_file = rospy.get_param('~calibration_file', '')
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 90)

        # Shutdown flag
        self.is_shutdown = False

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.compressed_pub = rospy.Publisher('/camera/image_compressed', CompressedImage, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

        # Open camera with V4L2 backend (avoid GStreamer issues)
        rospy.loginfo(f"Opening camera device {self.device_id}...")
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            rospy.logfatal(f"Cannot open camera device {self.device_id}")
            rospy.logfatal("Make sure DroidCam is connected:")
            rospy.logfatal("  1. Run: droidcam-cli <phone_ip> 4747")
            rospy.logfatal("  2. Or connect via USB and run: droidcam-cli -a")
            raise RuntimeError("Cannot open camera")

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Get actual resolution
        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        rospy.loginfo(f"Camera opened: {self.actual_width}x{self.actual_height} @ {actual_fps} FPS")

        # Load camera calibration
        self.camera_info_msg = self._load_camera_info()

        # JPEG encoding parameters
        self.encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        # Create GUI window if enabled (with larger size)
        if self.show_gui:
            cv2.namedWindow("Camera Preview", cv2.WINDOW_NORMAL)
            # Set window size (1280x720 or 1.5x camera resolution)
            window_width = max(1280, int(self.actual_width * 1.5))
            window_height = max(720, int(self.actual_height * 1.5))
            cv2.resizeWindow("Camera Preview", window_width, window_height)
            rospy.loginfo(f"Camera preview window: {window_width}x{window_height}")

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

        # Timer for capture loop
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.fps), self.capture_callback)

        rospy.loginfo("Camera node started")

    def _load_camera_info(self) -> CameraInfo:
        """Load camera calibration from YAML file or use estimated parameters."""
        info = CameraInfo()
        info.width = self.actual_width
        info.height = self.actual_height

        if self.calibration_file and Path(self.calibration_file).exists():
            try:
                # Load from OpenCV YAML calibration file
                fs = cv2.FileStorage(self.calibration_file, cv2.FILE_STORAGE_READ)
                if fs.isOpened():
                    K = fs.getNode("camera_matrix").mat()
                    D = fs.getNode("distortion_coefficients").mat().flatten()
                    fs.release()

                    if K is not None and D is not None:
                        info.K = K.flatten().tolist()
                        info.D = D.tolist()
                        info.distortion_model = "plumb_bob"

                        # P = K [I | 0]
                        P = np.zeros((3, 4))
                        P[:3, :3] = K
                        info.P = P.flatten().tolist()
                        info.R = np.eye(3).flatten().tolist()

                        rospy.loginfo(f"Loaded calibration from {self.calibration_file}")
                        return info
            except Exception as e:
                rospy.logwarn(f"Failed to load calibration: {e}")

        # Use estimated parameters
        rospy.logwarn("Using ESTIMATED camera parameters (run calibration for accuracy!)")
        return self._get_estimated_camera_info()

    def _get_estimated_camera_info(self) -> CameraInfo:
        """Generate estimated camera parameters for typical webcam/phone camera."""
        info = CameraInfo()
        info.width = self.actual_width
        info.height = self.actual_height

        # Estimate focal length: ~0.8-1.0 * width for typical phone camera FOV (~60-80 degrees)
        fx = fy = 0.9 * self.actual_width
        cx = self.actual_width / 2.0
        cy = self.actual_height / 2.0

        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]])

        info.K = K.flatten().tolist()
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assume no distortion
        info.distortion_model = "plumb_bob"

        P = np.zeros((3, 4))
        P[:3, :3] = K
        info.P = P.flatten().tolist()
        info.R = np.eye(3).flatten().tolist()

        return info

    def capture_callback(self, event):
        """Timer callback: capture frame and publish."""
        if self.is_shutdown:
            return

        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn_throttle(1.0, "Failed to capture frame")
            return

        timestamp = rospy.Time.now()

        # Show preview if enabled
        if self.show_gui and not self.is_shutdown:
            cv2.imshow("Camera Preview", frame)
            cv2.waitKey(1)

        # Publish raw image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = "camera_link"
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to publish raw image: {e}")

        # Publish compressed image
        try:
            ok, buf = cv2.imencode('.jpg', frame, self.encode_params)
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = "camera_link"
                compressed_msg.format = "jpeg"
                compressed_msg.data = buf.tobytes()
                self.compressed_pub.publish(compressed_msg)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to publish compressed image: {e}")

        # Publish camera info
        self.camera_info_msg.header.stamp = timestamp
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_pub.publish(self.camera_info_msg)

    def shutdown(self):
        """Clean up resources."""
        rospy.loginfo("Shutting down camera node...")
        self.is_shutdown = True

        # Stop timer
        if hasattr(self, 'timer'):
            self.timer.shutdown()

        # Release camera
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

        # Destroy all OpenCV windows
        cv2.destroyAllWindows()

        # Wait a bit to ensure windows are closed
        cv2.waitKey(1)
        rospy.loginfo("Camera node shutdown complete")


def main():
    try:
        node = CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"Camera node error: {e}")
        raise


if __name__ == '__main__':
    main()
