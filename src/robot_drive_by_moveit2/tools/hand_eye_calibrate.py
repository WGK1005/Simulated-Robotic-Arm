#!/usr/bin/env python3
"""
hand_eye_calibrate.py - eye-to-hand hand-eye calibration.

Configuration
  camera : FIXED in the workspace (bolted to the chassis / a stand)
  board  : AprilTag rigidly mounted on the robot end effector (tool_link)

It records, for every pose you command:
  A_i = T_base_tool   (from TF, the robot's forward kinematics)
  B_i = T_cam_tag     (from AprilTag detection + the known tag size)

and solves A_i X = Z B_i for
  Z = T_base_cam      <- the answer: where the camera sits in base_link
  X = T_tool_tag      (a by-product, the board's mounting transform)

Usage
  # terminal 1: robot + camera driver running
  # terminal 2:
  python3 hand_eye_calibrate.py --tag-size 0.065 --tag-id 0

  Move the arm to a pose -> wait until the terminal reports the tag is
  visible -> press Enter to capture. 15-20 well spread poses are plenty.
  Vary position AND orientation a lot; translation-only motion is useless
  for hand-eye calibration.

Result is printed and written to hand_eye_result.yaml.

Needs: rclpy, cv_bridge, tf2_ros, numpy, opencv with the aruco module
  python3 -c "import cv2; print(cv2.__version__, hasattr(cv2,'aruco'))"
if aruco is missing:
  pip install opencv-contrib-python   (watch out: may upgrade numpy)
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

import cv2
from cv_bridge import CvBridge

import tf2_ros

from tag_pose import TagDetector

BASE_FRAME = "base_link"


# --------------------------------------------------------------------------
# small 3D maths helpers (avoid extra dependencies)
# --------------------------------------------------------------------------
def quat_to_mat(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def mat_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def inv(T):
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


# --------------------------------------------------------------------------
class HandEyeCalibrator(Node):
    def __init__(self, args):
        super().__init__("hand_eye_calibrate")
        self.args = args
        self.bridge = CvBridge()
        self.frame = None
        self.K = None
        self.D = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()

        # detection + single-tag pose live in tag_pose.py, which is verified
        # by test_tag_pose.py and handles OpenCV < 4.7 as well as >= 4.7
        self.tag_detector = TagDetector(args.tag_size, args.tag_id)

        self.create_subscription(Image, args.image, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.info, self.on_info, 10)

    # ---------------------------------------------------------------- callbacks
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self.lock:
            self.frame = frame

    def on_info(self, msg):
        with self.lock:
            if self.K is None:
                self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
                self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    # ------------------------------------------------------------------ detect
    def tag_pose(self):
        """Return (T_cam_tag, message). T_cam_tag is 4x4 or None."""
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            K = None if self.K is None else self.K.copy()
            D = None if self.D is None else self.D.copy()

        if frame is None:
            return None, "no image yet"
        if K is None:
            return None, "no camera_info yet"

        return self.tag_detector.pose(frame, K, D)

    def tool_pose(self):
        """T_base_tool from TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame, self.args.tool_frame, rclpy.time.Time())
        except Exception as exc:
            return None, str(exc)
        tr = tf.transform.translation
        q = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T, "ok"


# --------------------------------------------------------------------------
def solve(samples):
    """samples: list of (A_i = T_base_tool, B_i = T_cam_tag).

    Eye-to-hand: camera fixed, board riding on the tool. The relation is

        A_i . X = Z . B_i        X = T_tool_tag (board mount, const)
                                 Z = T_base_cam (WHAT WE WANT, const)

    cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam,
    t_target2cam) expects eye-in-hand inputs. Feeding the INVERTED robot
    poses together with the raw detections

        gripper2base <- inv(T_base_tool)  = T_tool_base
        target2cam   <- T_cam_tag

    makes it return Z = T_base_cam directly. Verified against synthetic
    ground truth (tools/test_handeye_solve.py): exact recovery with PARK,
    TSAI and HORAUD; passing the non-inverted poses returns the wrong thing.
    """
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for A, B in samples:
        A_inv = inv(A)
        R_g2b.append(A_inv[:3, :3])
        t_g2b.append(A_inv[:3, 3].reshape(3, 1))
        R_t2c.append(B[:3, :3])
        t_t2c.append(B[:3, 3].reshape(3, 1))

    methods = [cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_TSAI,
               cv2.CALIB_HAND_EYE_HORAUD, cv2.CALIB_HAND_EYE_ANDREFF,
               cv2.CALIB_HAND_EYE_DANIILIDIS]
    results = []
    for m in methods:
        try:
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
        except TypeError:
            # older python bindings want method positionally
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, None, None, m)
        except Exception:
            continue
        Z = np.eye(4)
        Z[:3, :3] = R
        Z[:3, 3] = t.flatten()
        results.append((m, Z))
    if not results:
        return None, "calibrateHandEye failed for every method"

    # pick the solution that makes T_tool_tag most consistent across samples
    best = None
    for m, Z in results:
        Xs = [inv(A) @ Z @ B for A, B in samples]
        t_spread = np.std([X[:3, 3] for X in Xs], axis=0).max()
        R_ref = Xs[0][:3, :3]
        rot_spread = max(np.linalg.norm(cv2.Rodrigues(X[:3, :3] @ R_ref.T)[0])
                         for X in Xs)
        score = t_spread + 0.1 * rot_spread
        if best is None or score < best[0]:
            best = (score, m, Z, t_spread, rot_spread)

    _, method, Z, t_spread, rot_spread = best
    return Z, (method, t_spread, rot_spread)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag-size", type=float, default=0.065,
                    help="AprilTag black square edge length in METRES (default 0.065)")
    ap.add_argument("--tag-id", type=int, default=0, help="which tag id to use")
    ap.add_argument("--samples", type=int, default=15,
                    help="how many poses to record (default 15)")
    ap.add_argument("--image", default="/camera/color/image_raw")
    ap.add_argument("--info", default="/camera/color/camera_info")
    ap.add_argument("--base-frame", default=BASE_FRAME)
    ap.add_argument("--tool-frame", default="tool_link")
    ap.add_argument("--out", default="hand_eye_result.yaml")
    args = ap.parse_args()

    if not hasattr(cv2, "aruco"):
        raise SystemExit(
            "this opencv has no aruco module.\n"
            "check with: python3 -c \"import cv2; print(cv2.__version__, hasattr(cv2,'aruco'))\"\n"
            "install with: pip install opencv-contrib-python")

    print("AprilTag 36h11, tag id %d, edge %.1f mm"
          % (args.tag_id, args.tag_size * 1000))
    print("config    : eye-to-hand (camera fixed in %s, board on %s)"
          % (args.base_frame, args.tool_frame))
    print("image     : %s" % args.image)
    print("solving   : T_%s_camera  (where the camera sits in the base frame)"
          % args.base_frame)
    print("            using %d poses" % args.samples)

    rclpy.init()
    node = HandEyeCalibrator(args)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    print("waiting for camera + TF ...")
    for _ in range(100):
        ok_img = node.frame is not None
        T, _m = node.tool_pose()
        if ok_img and T is not None:
            break
        time.sleep(0.1)
    else:
        print("!! camera or TF not ready. is the driver running? is TF up?")
        print("   image:", node.frame is not None, " TF:", node.tool_pose()[0] is not None)

    samples = []
    poses = []
    while len(samples) < args.samples:
        try:
            input("\n[%d/%d] move the arm, then press Enter to capture > "
                  % (len(samples) + 1, args.samples))
        except EOFError:
            break

        A, msg_a = node.tool_pose()
        B, msg_b = node.tag_pose()
        if A is None:
            print("   FAIL tf    :", msg_a)
            continue
        if B is None:
            print("   FAIL tag   :", msg_b)
            continue

        pos = A[:3, 3]
        if poses:
            d = min(np.linalg.norm(pos - p) for p in poses)
            if d < 0.03:
                print("   skipped   : only %.0f mm from an earlier pose, move more"
                      % (d * 1000))
                continue
        poses.append(pos)
        samples.append((A, B))
        print("   captured  : %s   base_tool xyz = [%.3f %.3f %.3f]"
              % (msg_b, pos[0], pos[1], pos[2]))

    print("\ncollected %d samples, solving ..." % len(samples))
    if len(samples) < 5:
        print("!! need at least 5 samples")
        executor.shutdown()
        rclpy.shutdown()
        return

    Z, info = solve(samples)
    if Z is None:
        print("!! solve failed:", info)
        executor.shutdown()
        rclpy.shutdown()
        return

    method, t_spread, rot_spread = info
    t = Z[:3, 3]
    rpy = mat_to_rpy(Z[:3, :3])
    quat = mat_to_quat(Z[:3, :3])

    print("\n" + "=" * 62)
    print("RESULT  -  camera pose in the %s frame" % args.base_frame)
    print("=" * 62)
    print("method            : %s" % method)
    print("translation xyz   : [%.4f, %.4f, %.4f] m" % (t[0], t[1], t[2]))
    print("rotation rpy      : [%.4f, %.4f, %.4f] rad (%.2f, %.2f, %.2f deg)"
          % (rpy[0], rpy[1], rpy[2],
             math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2])))
    print("quaternion xyzw   : [%.5f, %.5f, %.5f, %.5f]"
          % (quat[0], quat[1], quat[2], quat[3]))
    print()
    print("consistency check (T_tool_tag should be identical for every pose)")
    print("  translation spread : %.4f m   (want < 0.005)" % t_spread)
    print("  rotation spread    : %.4f rad (want < 0.01)" % rot_spread)
    if t_spread > 0.01 or rot_spread > 0.02:
        print("  -> poor. recapture with more varied poses (tilt the wrist!).")
    else:
        print("  -> good.")

    with open(args.out, "w") as f:
        f.write("# eye-to-hand calibration result\n")
        f.write("# camera pose in the %s frame\n" % args.base_frame)
        f.write("method: %s\n" % method)
        f.write("tag_size_m: %g\n" % args.tag_size)
        f.write("tag_id: %d\n" % args.tag_id)
        f.write("samples: %d\n" % len(samples))
        f.write("translation_xyz_m: [%f, %f, %f]\n" % (t[0], t[1], t[2]))
        f.write("quaternion_xyzw: [%f, %f, %f, %f]\n"
                % (quat[0], quat[1], quat[2], quat[3]))
        f.write("rpy_rad: [%f, %f, %f]\n" % (rpy[0], rpy[1], rpy[2]))
        f.write("translation_spread_m: %f\n" % t_spread)
        f.write("rotation_spread_rad: %f\n" % rot_spread)
    print("\nwritten:", args.out)

    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
