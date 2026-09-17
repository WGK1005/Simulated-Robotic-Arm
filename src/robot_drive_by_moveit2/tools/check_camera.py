#!/usr/bin/env python3
"""
check_camera.py - verify a ROS 2 camera is publishing and ready for
intrinsic calibration.

It checks, in order:
  1. which image / camera_info topics exist and their publish rates
  2. one frame from the colour topic - resolution, encoding, whether it is
     actually non-black (a black frame usually means the wrong topic or the
     stream has not started)
  3. camera_info - are K (intrinsics) and D (distortion) populated, or zero?
  4. optionally: can the calibration pattern be detected in that frame?
     --size 8x6        chessboard with 8x6 inner corners
     --aruco           any ArUco / AprilTag marker (needs cv2.aruco)
     Results are saved as a PNG you can look at.

Usage
  python3 check_camera.py
  python3 check_camera.py --image /camera/color/image_raw --info /camera/color/camera_info
  python3 check_camera.py --size 8x6 --square 0.025
  python3 check_camera.py --aruco
  python3 check_camera.py --save /tmp/frame.png

Needs: rclpy, sensor_msgs, cv_bridge, numpy, opencv
  sudo apt install -y ros-humble-cv-bridge python3-opencv
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

OK = "[ OK ]"
FAIL = "[FAIL]"
WARN = "[WARN]"


class CameraCheck(Node):
    def __init__(self, image_topic, info_topic):
        super().__init__("check_camera")
        self.bridge = CvBridge() if CvBridge else None
        self.image = None
        self.info = None
        self.image_count = 0
        self.first_stamp = None
        self.last_stamp = None

        self.create_subscription(Image, image_topic, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.image_topic = image_topic
        self.info_topic = info_topic

    def on_image(self, msg):
        self.image = msg
        self.image_count += 1
        now = time.time()
        if self.first_stamp is None:
            self.first_stamp = now
        self.last_stamp = now

    def on_info(self, msg):
        self.info = msg

    def spin_for(self, seconds):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def rate(self):
        if self.image_count < 2 or not self.first_stamp or not self.last_stamp:
            return 0.0
        dt = self.last_stamp - self.first_stamp
        return (self.image_count - 1) / dt if dt > 0 else 0.0


def list_camera_topics(node):
    print("camera-related topics:")
    found = []
    for name, types in sorted(node.get_topic_names_and_types()):
        if "image" in name or "camera_info" in name:
            print("   %-45s %s" % (name, ", ".join(types)))
            found.append(name)
    if not found:
        print("   (none) - is the camera driver running?")
    return found


def autodetect(topics):
    """Pick a colour image topic and its camera_info."""
    img = info = None
    for t in topics:
        if t.endswith("image_raw") or t.endswith("/image"):
            if img is None:
                img = t
    for t in topics:
        if t.endswith("camera_info"):
            base = t[: -len("camera_info")]
            if img and base in img:
                info = t
                break
    if info is None:
        for t in topics:
            if t.endswith("camera_info"):
                info = t
                break
    return img, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="image topic (default: auto-detect)")
    ap.add_argument("--info", help="camera_info topic (default: auto-detect)")
    ap.add_argument("--wait", type=float, default=10.0,
                    help="seconds to wait for the first frame")
    ap.add_argument("--save", default="/tmp/camera_check.png",
                    help="where to write the grabbed frame")
    ap.add_argument("--size", help="chessboard inner corners, e.g. 8x6")
    ap.add_argument("--square", type=float, default=0.025,
                    help="chessboard square size in metres")
    ap.add_argument("--aruco", action="store_true",
                    help="also try to detect ArUco/AprilTag markers")
    args = ap.parse_args()

    if CvBridge is None:
        print(FAIL, "cv_bridge not found. Install it with:")
        print("      sudo apt install -y ros-humble-cv-bridge python3-opencv")
        return 2

    rclpy.init()
    probe = Node("camera_check_probe")
    topics = list_camera_topics(probe)
    image_topic, info_topic = args.image, args.info

    if image_topic is None or info_topic is None:
        auto_img, auto_info = autodetect(topics)
        image_topic = image_topic or auto_img
        info_topic = info_topic or auto_info
    probe.destroy_node()

    if not image_topic:
        print(FAIL, "no image topic found - start the camera driver first")
        rclpy.shutdown()
        return 2
    print()
    print("using image topic :", image_topic)
    print("using info  topic :", info_topic)

    node = CameraCheck(image_topic, info_topic)
    print("waiting up to %.1fs for a frame ..." % args.wait)
    node.spin_for(args.wait)

    ok = True

    # ---- 1. frames arriving ------------------------------------------------
    if node.image is None:
        print(FAIL, "no image received on", image_topic)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    node.spin_for(1.0)
    print(OK, "receiving images, ~%.1f Hz (%d frames)"
          % (node.rate(), node.image_count))

    # ---- 2. frame properties ----------------------------------------------
    msg = node.image
    print(OK, "resolution %dx%d, encoding '%s'"
          % (msg.width, msg.height, msg.encoding))

    frame = node.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    mean = float(np.mean(frame))
    print("     mean brightness = %.1f / 255" % mean)
    if mean < 3:
        print(WARN, "frame looks almost black - wrong topic, lens cap, or the")
        print("      stream has not really started")
        ok = False
    elif mean > 250:
        print(WARN, "frame looks fully saturated - lower the exposure")

    # ---- 3. camera_info ----------------------------------------------------
    if node.info is None:
        print(FAIL, "no camera_info received on", info_topic)
        print("      without it you cannot compare or store a calibration")
        ok = False
    else:
        K = np.array(node.info.k).reshape(3, 3)
        D = np.array(node.info.d)
        print(OK, "camera_info: %dx%d, distortion model '%s', %d coeffs"
              % (node.info.width, node.info.height,
                 node.info.distortion_model, len(D)))
        print("     K = [[%.2f, %.2f, %.2f]," % tuple(K[0]))
        print("          [%.2f, %.2f, %.2f]," % tuple(K[1]))
        print("          [%.2f, %.2f, %.2f]]" % tuple(K[2]))
        print("     D =", np.round(D, 5))
        if K[0, 0] == 0.0 or K[1, 1] == 0.0:
            print(WARN, "K has zero focal length - the driver is not publishing")
            print("      real intrinsics yet (that is exactly what calibration fixes)")
        else:
            print("     -> these are the values currently in use")

    # ---- 4. pattern detection ---------------------------------------------
    if args.size:
        try:
            import cv2
        except ImportError:
            print(WARN, "opencv missing, skipping pattern detection")
            cv2 = None
        if cv2 is not None:
            cols, rows = (int(v) for v in args.size.lower().split("x"))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                     cv2.CALIB_CB_NORMALIZE_IMAGE |
                     cv2.CALIB_CB_FAST_CHECK)
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
            if found:
                print(OK, "chessboard %dx%d FOUND in this frame" % (cols, rows))
                cv2.drawChessboardCorners(frame, (cols, rows), corners, found)
            else:
                print(WARN, "chessboard %dx%d not found in this frame" % (cols, rows))
                print("      (normal if the board is not in view right now)")

    if args.aruco:
        try:
            import cv2
            aruco = cv2.aruco
        except (ImportError, AttributeError):
            print(WARN, "cv2.aruco not available - install opencv-contrib-python")
            print("      pip install opencv-contrib-python")
            aruco = None
        if aruco is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                print(OK, "AprilTag/ArUco markers detected: %s"
                      % [int(i) for i in ids.flatten()])
                aruco.drawDetectedMarkers(frame, corners, ids)
            else:
                print(WARN, "no AprilTag/ArUco marker found in this frame")

    # ---- save --------------------------------------------------------------
    try:
        import cv2
        cv2.imwrite(args.save, frame)
        print(OK, "frame saved to", args.save)
        print("     open it to see what the camera sees")
    except ImportError:
        print(WARN, "opencv missing, cannot save the frame")

    print()
    print("RESULT:", "camera looks READY for calibration" if ok
          else "fix the problems marked " + FAIL + " before calibrating")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
