#!/usr/bin/env python3
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ArucoIntrinsicsCalibrator:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.latest_gray = None
        self.latest_bgr = None
        self.image_size = None

        self.samples_obj = []
        self.samples_img = []

        self.ws_centers = self._workspace_marker_centers()
        self.dict = self._make_aruco_dictionary()
        self.params = self._make_detector_params()
        self.detector = self._make_detector()

        self.sub = rospy.Subscriber(args.image_topic, Image, self._cb, queue_size=1)

    def _workspace_marker_centers(self):
        # Match weld_line_vision_node.cpp defaults.
        marker_half = self.args.marker_size_m * 0.5

        tl = (marker_half + self.args.ws_w1_margin_offset, marker_half + self.args.ws_h1_margin_offset)
        tr = (self.args.paper_width_m - marker_half - self.args.ws_w2_margin_offset, marker_half + self.args.ws_h2_margin_offset)
        bl = (marker_half + self.args.ws_w3_margin_offset, self.args.paper_height_m - marker_half - self.args.ws_h3_margin_offset)
        br = (
            self.args.paper_width_m - marker_half - self.args.ws_w4_margin_offset,
            self.args.paper_height_m - marker_half - self.args.ws_h4_margin_offset,
        )

        return {
            self.args.id_tl: np.array([tl[0], tl[1], 0.0], dtype=np.float32),
            self.args.id_tr: np.array([tr[0], tr[1], 0.0], dtype=np.float32),
            self.args.id_bl: np.array([bl[0], bl[1], 0.0], dtype=np.float32),
            self.args.id_br: np.array([br[0], br[1], 0.0], dtype=np.float32),
        }

    @staticmethod
    def _make_aruco_dictionary():
        aruco = cv2.aruco
        if hasattr(aruco, "getPredefinedDictionary"):
            return aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        return aruco.Dictionary_get(aruco.DICT_4X4_50)

    @staticmethod
    def _make_detector_params():
        aruco = cv2.aruco
        if hasattr(aruco, "DetectorParameters_create"):
            return aruco.DetectorParameters_create()
        if hasattr(aruco, "DetectorParameters"):
            return aruco.DetectorParameters()
        raise RuntimeError("OpenCV aruco DetectorParameters API not available.")

    def _make_detector(self):
        aruco = cv2.aruco
        if hasattr(aruco, "ArucoDetector"):
            return aruco.ArucoDetector(self.dict, self.params)
        return None

    def _cb(self, msg):
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", str(e))
                return

        self.latest_gray = gray
        self.image_size = (gray.shape[1], gray.shape[0])
        self.latest_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    @staticmethod
    def _center(corners4):
        c = np.mean(np.asarray(corners4, dtype=np.float32).reshape(4, 2), axis=0)
        return np.array([float(c[0]), float(c[1])], dtype=np.float32)

    def _detect(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        out = {}
        if ids is None:
            return out, corners, ids
        ids = ids.flatten().tolist()
        for i, mid in enumerate(ids):
            if mid in self.ws_centers:
                out[mid] = self._center(corners[i])
        return out, corners, ids

    def _draw_overlay(self, vis, found, corners, ids):
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, np.array(ids, dtype=np.int32))

        y = 28
        step = 26
        lines = [
            "Aruco Intrinsics Calibrator",
            "Need IDs: %d,%d,%d,%d" % (self.args.id_tl, self.args.id_tr, self.args.id_bl, self.args.id_br),
            "Detected needed IDs: %s" % ("none" if not found else ",".join(map(str, sorted(found.keys())))),
            "Samples: %d (recommend >= 25)" % len(self.samples_obj),
            "c: capture sample (when 4 IDs visible)",
            "k: calibrate + save YAML",
            "q: quit",
        ]
        for line in lines:
            cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            y += step

    def _capture_sample(self, found):
        required = [self.args.id_tl, self.args.id_tr, self.args.id_bl, self.args.id_br]
        if any(mid not in found for mid in required):
            rospy.logwarn("Cannot capture: all workspace marker IDs must be visible.")
            return

        obj = np.array([self.ws_centers[mid] for mid in required], dtype=np.float32)
        img = np.array([found[mid] for mid in required], dtype=np.float32)

        self.samples_obj.append(obj)
        self.samples_img.append(img)
        rospy.loginfo("Captured sample %d", len(self.samples_obj))

    def _calibrate_and_save(self):
        if len(self.samples_obj) < self.args.min_samples:
            rospy.logwarn("Need at least %d samples, currently %d", self.args.min_samples, len(self.samples_obj))
            return
        if self.image_size is None:
            rospy.logwarn("No image received yet.")
            return

        flags = 0
        ret, K, D, _, _ = cv2.calibrateCamera(
            self.samples_obj,
            self.samples_img,
            self.image_size,
            None,
            None,
            flags=flags,
        )

        if not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
            rospy.logerr("Calibration produced invalid intrinsics. Try more diverse samples.")
            return

        if D is None:
            D = np.zeros((1, 5), dtype=np.float64)

        D = np.asarray(D, dtype=np.float64).reshape(-1)
        if D.size < 5:
            D = np.pad(D, (0, 5 - D.size), mode="constant")
        D5 = D[:5]

        out = self.args.output
        os.makedirs(os.path.dirname(out), exist_ok=True)

        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])

        yaml_txt = f"""image_width: {self.image_size[0]}
image_height: {self.image_size[1]}
camera_name: {self.args.camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{fx:.10f}, 0.0, {cx:.10f}, 0.0, {fy:.10f}, {cy:.10f}, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{D5[0]:.10f}, {D5[1]:.10f}, {D5[2]:.10f}, {D5[3]:.10f}, {D5[4]:.10f}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{fx:.10f}, 0.0, {cx:.10f}, 0.0, 0.0, {fy:.10f}, {cy:.10f}, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
        with open(out, "w", encoding="ascii") as f:
            f.write(yaml_txt)

        rospy.loginfo("Calibration RMS error: %.6f", float(ret))
        rospy.loginfo("Saved camera calibration YAML: %s", out)
        rospy.loginfo("fx=%.3f fy=%.3f cx=%.3f cy=%.3f", fx, fy, cx, cy)

    def run(self):
        rospy.loginfo("Waiting for images on %s ...", self.args.image_topic)

        while not rospy.is_shutdown() and self.latest_gray is None:
            time.sleep(0.05)

        while not rospy.is_shutdown():
            gray = self.latest_gray.copy()
            vis = self.latest_bgr.copy()

            found, corners, ids = self._detect(gray)
            self._draw_overlay(vis, found, corners, ids)

            cv2.imshow("aruco_intrinsics_calibrator", vis)
            key = cv2.waitKey(10) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                self._capture_sample(found)
            if key == ord("k"):
                self._calibrate_and_save()

        cv2.destroyAllWindows()


def parse_args():
    p = argparse.ArgumentParser(description="Calibrate camera intrinsics from workspace ArUco IDs 0/1/2/3.")
    p.add_argument("--image-topic", default="/droidcam/image_raw")
    p.add_argument("--output", default="/root/.ros/camera_info/droidcam.yaml")
    p.add_argument("--camera-name", default="droidcam")
    p.add_argument("--min-samples", type=int, default=20)

    p.add_argument("--paper-width-m", type=float, default=0.210)
    p.add_argument("--paper-height-m", type=float, default=0.297)
    p.add_argument("--marker-size-m", type=float, default=0.06)

    p.add_argument("--ws-w1-margin-offset", type=float, default=0.015)
    p.add_argument("--ws-h1-margin-offset", type=float, default=0.018)
    p.add_argument("--ws-w2-margin-offset", type=float, default=0.015)
    p.add_argument("--ws-h2-margin-offset", type=float, default=0.018)
    p.add_argument("--ws-w3-margin-offset", type=float, default=0.015)
    p.add_argument("--ws-h3-margin-offset", type=float, default=0.018)
    p.add_argument("--ws-w4-margin-offset", type=float, default=0.015)
    p.add_argument("--ws-h4-margin-offset", type=float, default=0.018)

    p.add_argument("--id-tl", type=int, default=0)
    p.add_argument("--id-tr", type=int, default=1)
    p.add_argument("--id-bl", type=int, default=2)
    p.add_argument("--id-br", type=int, default=3)
    return p.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    rospy.init_node("aruco_intrinsics_calibrator", anonymous=True)
    args = parse_args()
    calib = ArucoIntrinsicsCalibrator(args)
    calib.run()


if __name__ == "__main__":
    main()
