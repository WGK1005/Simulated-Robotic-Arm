#!/usr/bin/env python3
"""
point_register.py - eye-to-hand extrinsic calibration by POINT CONTACT.

No fiducial marker is needed (no AprilTag, no ArUco, no checkerboard).

Idea
  A pointer with a small brightly coloured ball on its tip is mounted on the
  robot.  You jog the robot so the tip touches N points spread through the
  workspace.  For every point the SAME physical location is observed twice:

    p_cam   - the ball centre, from the colour image + the ALIGNED depth image
    p_base  - the pointer tip, from TF (forward kinematics) + the tip offset

  and all N points are solved at once for

    p_base = R * p_cam + t

  by SVD (Kabsch / Umeyama).  R and t are exactly T_base_cam_optical.

Why this works without a marker board
  A marker board is only ever a way of measuring "where is this physical point
  in the camera frame".  Touching a point with a coloured tip measures the same
  thing directly, and it also gives the robot side of the correspondence for
  free from forward kinematics.

Depth alignment
  If the depth image is already registered to colour (its frame_id equals the
  colour frame_id) the marker blob indexes it directly - the most accurate
  case.

  If it is not registered, the colour pixel is mapped to the matching depth
  pixel here, using the depth camera intrinsics and the TF between the two
  optical frames.  That is the same computation depth_image_proc performs, so
  no extra node or package is needed.

Usage
  # terminal 1 : robot bringup + camera driver
  # terminal 2 :
  python3 point_register.py

  Jog the pointer tip onto a point -> wait for "marker OK" -> press Enter.
  Repeat for 8 or more points spread over the whole working volume.

  8-12 points is the sweet spot.  Spread them in X, Y AND Z - if every point
  lies on the table surface the solve cannot pin down the depth direction.

  Points you touch must be reachable with the tip AND visible to the camera
  at the same time.  Type "t" instead of Enter to test a capture without
  recording it.

Result is printed and written to point_register_result.yaml.

Self test (no hardware, no ROS needed)
  python3 point_register.py --selftest

Needs: rclpy, cv_bridge, tf2_ros, numpy, opencv
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

import cv2

BASE_FRAME = "base_link"
OPTICAL_FRAME = "camera_color_optical_frame"


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
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    o = np.eye(4)
    o[:3, :3] = R.T
    o[:3, 3] = -R.T @ t
    return o


# --------------------------------------------------------------------------
# the solve
# --------------------------------------------------------------------------
def kabsch(P, Q):
    """least-squares rigid transform with  p_base = R * p_cam + t.

    P : (n,3) points in the camera optical frame
    Q : (n,3) the same points in the base frame

    Closed form (Kabsch 1976 / Umeyama 1991):
      centre both sets, then the rotation that best aligns them is
      R = V * diag(1, 1, det(V U^T)) * U^T  from the SVD of the covariance.
      The determinant factor is what stops the answer coming back as a
      reflection instead of a rotation.
    """
    P = np.asarray(P, dtype=float)
    Q = np.asarray(Q, dtype=float)
    c_p = P.mean(axis=0)
    c_q = Q.mean(axis=0)
    A = P - c_p
    B = Q - c_q

    H = A.T @ B
    U, _s, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = c_q - R @ c_p
    return R, t


def solve_point_registration(p_cam, p_base):
    """solve T_base_cam plus a full quality report.

    Returns (T, info).  T is None when the data cannot support a solve.
    """
    P = np.asarray(p_cam, dtype=float)
    Q = np.asarray(p_base, dtype=float)
    n = len(P)

    if n < 3:
        return None, "need at least 3 points (4+ recommended, 8-12 ideal)"

    # how well do the touched points actually span 3D space?
    A = P - P.mean(axis=0)
    s = np.linalg.svd(A, compute_uv=False)
    if s[0] < 1e-9:
        return None, "all touch points are identical, nothing to solve"
    span_ratio = float(s[2] / s[0])

    R, t = kabsch(P, Q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    pred = (P @ R.T) + t
    err = np.linalg.norm(pred - Q, axis=1)

    # leave-one-out: solve without each point and predict it back.  This is
    # the honest accuracy number, because it is measured on data that did not
    # take part in the fit.
    loo = []
    if n >= 5:
        for i in range(n):
            keep = np.ones(n, dtype=bool)
            keep[i] = False
            R_i, t_i = kabsch(P[keep], Q[keep])
            loo.append(float(np.linalg.norm((R_i @ P[i] + t_i) - Q[i])))

    info = {
        "n": n,
        "residual_mm": err * 1000.0,
        "rms_mm": float(np.sqrt(np.mean(err ** 2)) * 1000.0),
        "max_mm": float(err.max() * 1000.0),
        "loo_rms_mm": float(np.sqrt(np.mean(np.square(loo))) * 1000.0) if loo else None,
        "loo_max_mm": float(np.max(loo) * 1000.0) if loo else None,
        "span_ratio": span_ratio,
        "singular_values": s,
        "min_pair_dist_m": _min_pair_distance(P),
        "det": float(np.linalg.det(R)),
    }
    return T, info


def _min_pair_distance(P):
    n = len(P)
    best = float("inf")
    for i in range(n):
        for j in range(i + 1, n):
            best = min(best, float(np.linalg.norm(P[i] - P[j])))
    return best


# --------------------------------------------------------------------------
# marker detection: coloured ball on the pointer tip
# --------------------------------------------------------------------------
def detect_marker(img_bgr, lo, hi, min_area, max_area):
    """largest blob inside the HSV window.

    Returns (centroid_uv, mask, message).  centroid_uv is None on failure.
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))

    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, mask, "no blob in the HSV window"

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None, mask, "blob too small (%.0f px, want >= %.0f)" % (area, min_area)
    if area > max_area:
        return None, mask, ("blob too big (%.0f px, want <= %.0f) - is something "
                            "else matching the colour?" % (area, max_area))

    M = cv2.moments(c)
    if M["m00"] <= 0:
        return None, mask, "degenerate blob"
    return (M["m10"] / M["m00"], M["m01"] / M["m00"]), mask, "area %.0f px" % area


def depth_from_mask(depth_m, mask, dmin, dmax, erode_px=2, min_px=8):
    """robust depth for a blob: erode the mask, then take the median.

    A single pixel is far too noisy and often lands on a depth edge, so the
    whole blob is used after trimming its border.
    """
    m = mask
    if erode_px > 0:
        k = np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8)
        e = cv2.erode(mask, k)
        if cv2.countNonZero(e) >= min_px:
            m = e

    vals = depth_m[m > 0]
    vals = vals[np.isfinite(vals)]
    vals = vals[(vals >= dmin) & (vals <= dmax)]
    if vals.size < min_px:
        return None, "no valid depth inside the marker (%d px)" % vals.size
    return float(np.median(vals)), "median of %d px" % vals.size


def depth_patch(depth_m, u, v, radius, dmin, dmax, min_px=5):
    """robust depth around a pixel, when there is no blob mask to use.

    Needed on the unaligned path, where the depth image has its own pixel grid
    and the marker blob does not exist there.
    """
    h, w = depth_m.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None, ("depth pixel (%.0f, %.0f) is outside the %dx%d depth image"
                      % (u, v, w, h))

    x0, x1 = max(0, ui - radius), min(w, ui + radius + 1)
    y0, y1 = max(0, vi - radius), min(h, vi + radius + 1)
    patch = depth_m[y0:y1, x0:x1].ravel()
    vals = patch[np.isfinite(patch)]
    vals = vals[(vals >= dmin) & (vals <= dmax)]
    if vals.size < min_px:
        return None, ("no valid depth near pixel (%.0f, %.0f), %d px"
                      % (u, v, vals.size))
    return float(np.median(vals)), "median of %d px" % vals.size


def map_pixel_to_depth(u, v, K_color, K_depth, T_color_depth, depth_m,
                       patch=3, dmin=0.15, dmax=2.0, guess=0.5):
    """depth seen along the COLOUR ray through pixel (u, v).

    Used when the depth image is not registered to colour.  With the two
    optical frames coincident (what most Astra drivers publish) this collapses
    to a pure intrinsics rescale:

        u_d = (u - cx_c) * fx_d / fx_c + cx_d        (same for v)

    The loop keeps it correct even when the driver does model a real baseline.

    Returns (Z_along_the_colour_axis, u_depth, v_depth, message).
    Z is None on failure.
    """
    fx_c, fy_c, cx_c, cy_c = (K_color[0, 0], K_color[1, 1],
                              K_color[0, 2], K_color[1, 2])
    fx_d, fy_d, cx_d, cy_d = (K_depth[0, 0], K_depth[1, 1],
                              K_depth[0, 2], K_depth[1, 2])

    # a colour-frame point on the ray is P_c = s * dir_c, which in the depth
    # frame reads  P_d(s) = s * dir_d + off_d.  So "the point sits at the
    # measured depth zd" is a closed-form update for s, not a guess:
    #     s = (zd - off_d[2]) / dir_d[2]
    T_depth_color = inv(T_color_depth)
    R_dc = T_depth_color[:3, :3]
    off_d = T_depth_color[:3, 3]
    dir_c = np.array([(u - cx_c) / fx_c, (v - cy_c) / fy_c, 1.0])
    dir_d = R_dc @ dir_c

    if abs(dir_d[2]) < 1e-9:
        return None, None, None, "the colour ray is parallel to the depth image plane"

    zc = guess
    ud = vd = None
    for _ in range(10):
        pd = zc * dir_d + off_d
        if pd[2] <= 1e-6:
            return None, None, None, "the colour ray points behind the depth camera"

        ud = fx_d * pd[0] / pd[2] + cx_d
        vd = fy_d * pd[1] / pd[2] + cy_d

        zd, msg = depth_patch(depth_m, ud, vd, patch, dmin, dmax)
        if zd is None:
            return None, ud, vd, msg

        zc_new = (zd - off_d[2]) / dir_d[2]
        if zc_new <= 0.0:
            return None, ud, vd, "the colour ray meets the surface behind the camera"

        moved = abs(zc_new - zc)
        zc = float(zc_new)
        if moved < 1e-5:
            break

    return zc, ud, vd, "depth pixel (%.0f, %.0f) -> %.3f m" % (ud, vd, zc)


def back_project(u, v, d, K):
    """pixel + depth -> 3D point in the camera OPTICAL frame.

    Optical frame convention: X right, Y down, Z forward (into the scene).
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    return np.array([(u - cx) * d / fx, (v - cy) * d / fy, d])


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------
class PointRegistrar:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from cv_bridge import CvBridge
        import tf2_ros

        self.args = args
        self.rclpy = rclpy
        self.node = Node("point_register")
        self.bridge = CvBridge()

        self.color = None
        self.color_frame = None
        self.color_stamp = None
        self.depth_raw = None
        self.depth_enc = None
        self.depth_m = None
        self.depth_frame = None
        self.depth_stamp = None
        self.K = None
        self.info_frame = None
        self.K_depth = None
        self.depth_info_frame = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.node.create_subscription(
            Image, args.image, self._on_color, qos_profile_sensor_data)
        self.node.create_subscription(
            Image, args.depth, self._on_depth, qos_profile_sensor_data)
        self.node.create_subscription(
            CameraInfo, args.info, self._on_info, 10)
        self.node.create_subscription(
            CameraInfo, args.depth_info, self._on_depth_info, 10)

    # -- callbacks ---------------------------------------------------------
    def _on_color(self, msg):
        try:
            self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.color_frame = msg.header.frame_id
            self.color_stamp = msg.header.stamp
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn("colour conversion failed: %s" % e)

    def _on_depth(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn("depth conversion failed: %s" % e)
            return
        enc = msg.encoding
        if enc in ("16UC1", "mono16"):
            m = img.astype(np.float32) * self.args.depth_scale
        elif enc in ("32FC1", "32FC1;") or enc.startswith("32F"):
            m = img.astype(np.float32)
        elif enc in ("16SC1",):
            m = img.astype(np.float32) * self.args.depth_scale
        else:
            self.node.get_logger().warn("unsupported depth encoding %r" % enc)
            return
        self.depth_raw = img
        self.depth_enc = enc
        self.depth_m = m
        self.depth_frame = msg.header.frame_id
        self.depth_stamp = msg.header.stamp

    def _on_info(self, msg):
        K = np.array(msg.k).reshape(3, 3)
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            return
        self.K = K
        self.info_frame = msg.header.frame_id

    def _on_depth_info(self, msg):
        K = np.array(msg.k).reshape(3, 3)
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            return
        self.K_depth = K
        self.depth_info_frame = msg.header.frame_id

    # -- state helpers -----------------------------------------------------
    def ready(self):
        return (self.color is not None and self.depth_m is not None
                and self.K is not None)

    def tool_tip_in_base(self):
        """pointer tip position in the base frame, from TF + the tip offset."""
        import rclpy
        try:
            tr = self.tf_buffer.lookup_transform(
                self.args.base_frame, self.args.tool_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            return None, str(e)

        tr_ = tr.transform
        R = quat_to_mat(tr_.rotation.x, tr_.rotation.y,
                        tr_.rotation.z, tr_.rotation.w)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tr_.translation.x, tr_.translation.y, tr_.translation.z]

        # the pointer sticks out of the tool frame along its own Z in the
        # common case, but the full 3-vector is accepted
        off = np.array(self.args.tip_offset, dtype=float)
        return R @ off + T[:3, 3], "ok"

    # -- capturing one point ----------------------------------------------
    def sample(self):
        """one capture.

        Returns (point_in_colour_optical_frame, message, debug_dict).
        point is None when this capture failed.
        """
        debug = {"uv": None, "mask": None}
        if self.color is None or self.depth_m is None or self.K is None:
            return None, "no colour/depth/intrinsics yet", debug

        uv, mask, msg_m = detect_marker(
            self.color, self.args.hsv_lower, self.args.hsv_upper,
            self.args.min_blob_area, self.args.max_blob_area)
        debug["uv"] = uv
        debug["mask"] = mask
        if uv is None:
            return None, "marker: %s" % msg_m, debug

        u, v = uv

        if self.depth_frame == self.color_frame:
            # depth already lives in the colour optical frame, so the blob mask
            # indexes it directly - the most accurate case
            d, msg_d = depth_from_mask(self.depth_m, mask, self.args.depth_min,
                                       self.args.depth_max)
            if d is None:
                return None, "depth: %s" % msg_d, debug
            return (back_project(u, v, d, self.K),
                    "aligned | %s | depth %.3f m" % (msg_m, d), debug)

        # depth has its own optical frame: work out which depth pixel the
        # colour pixel actually looks at
        p, msg_d = self._pixel_via_tf(u, v)
        if p is None:
            return None, "depth: %s" % msg_d, debug
        return p, "tf-mapped | %s | %s" % (msg_m, msg_d), debug

    def _pixel_via_tf(self, u, v):
        """read the depth seen along the colour ray, via the depth intrinsics
        and the TF between the two optical frames."""
        import rclpy

        if self.K_depth is None:
            return None, ("depth frame %r differs from colour frame %r and no "
                          "depth camera_info arrived on %s"
                          % (self.depth_frame, self.color_frame,
                             self.args.depth_info))

        try:
            tr = self.tf_buffer.lookup_transform(
                self.color_frame, self.depth_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            return None, ("no TF %s <- %s (%s)"
                          % (self.color_frame, self.depth_frame, e))

        tr_ = tr.transform
        T_color_depth = np.eye(4)
        T_color_depth[:3, :3] = quat_to_mat(tr_.rotation.x, tr_.rotation.y,
                                            tr_.rotation.z, tr_.rotation.w)
        T_color_depth[:3, 3] = [tr_.translation.x, tr_.translation.y,
                                tr_.translation.z]

        zc, ud, vd, msg = map_pixel_to_depth(
            u, v, self.K, self.K_depth, T_color_depth, self.depth_m,
            patch=self.args.depth_patch, dmin=self.args.depth_min,
            dmax=self.args.depth_max, guess=self.args.depth_guess)
        if zc is None:
            return None, ("at depth pixel (%.0f, %.0f) - %s" % (ud, vd, msg))

        return back_project(u, v, zc, self.K), msg

    # -- state helpers -----------------------------------------------------
    def alignment_warnings(self):
        """cheap checks for the mistakes that silently ruin this calibration."""
        w = []

        if self.info_frame and self.color_frame and self.info_frame != self.color_frame:
            w.append("colour camera_info frame %r != colour image frame %r -> the "
                     "intrinsics may belong to a different stream."
                     % (self.info_frame, self.color_frame))

        if (self.color_frame and self.depth_frame
                and self.color_frame != self.depth_frame
                and self.K_depth is None):
            w.append("depth frame %r differs from colour frame %r and no depth "
                     "camera_info has arrived, so the depth pixel cannot be "
                     "found. Check %s." % (self.depth_frame, self.color_frame,
                                           self.args.depth_info))
        return w

    def depth_mode(self):
        if self.depth_frame and self.color_frame == self.depth_frame:
            return "aligned to colour (blob mask used directly)"
        return ("NOT aligned - pixels mapped through TF and the depth "
                "intrinsics (equivalent to depth_image_proc registration)")


# --------------------------------------------------------------------------
# self test - synthetic data, no ROS, no hardware
# --------------------------------------------------------------------------
def selftest():
    rng = np.random.default_rng(11)

    T_true = np.eye(4)
    T_true[:3, :3] = cv2.Rodrigues(np.array([1.1, -0.4, 2.3]))[0]
    T_true[:3, 3] = [0.42, -0.13, 0.68]

    def make(r, n, noise_mm=0.0, planarity=1.0):
        """points spread over a volume of 0.5 m; planarity=0 -> flat plate.

        The flat plate sits at a constant distance from the camera, which is
        the worst case: it gives the solver nothing to pin the depth axis to.
        """
        P = r.uniform(-0.25, 0.25, size=(n, 3))
        P[:, 2] = r.uniform(-0.25, 0.25, n) * planarity + 0.7
        Q = (T_true[:3, :3] @ P.T).T + T_true[:3, 3]
        if noise_mm:
            Q = Q + r.normal(0, noise_mm / 1000.0, Q.shape)
        return P, Q

    print("ground-truth camera translation in base:", T_true[:3, 3])
    print()

    ok = True

    print("--- noise free ---")
    for n in (3, 4, 6, 8, 12, 20):
        P, Q = make(rng, n)
        T, info = solve_point_registration(P, Q)
        if T is None:
            print("n=%2d  FAILED: %s" % (n, info))
            ok = False
            continue
        dt = float(np.linalg.norm(T[:3, 3] - T_true[:3, 3]))
        dR = float(np.linalg.norm(cv2.Rodrigues(T[:3, :3] @ T_true[:3, :3].T)[0]))
        good = dt < 1e-6 and dR < 1e-6
        if not good:
            ok = False
        print("n=%2d  %s  |dt|=%.2e m  |dR|=%.2e rad   det(R)=%.3f"
              % (n, "OK " if good else "BAD", dt, dR, info["det"]))

    print()
    print("--- touch error on the robot side (8 points) ---")
    for mm in (0.0, 1.0, 2.0, 5.0, 10.0):
        P, Q = make(rng, 8, noise_mm=mm)
        T, info = solve_point_registration(P, Q)
        dt = float(np.linalg.norm(T[:3, 3] - T_true[:3, 3]))
        dR = float(np.linalg.norm(cv2.Rodrigues(T[:3, :3] @ T_true[:3, :3].T)[0]))
        print("  %5.1f mm per point -> |dt| = %6.1f mm   |dR| = %5.2f deg"
              "   fit rms %5.1f mm   LOO %5.1f mm"
              % (mm, dt * 1000, math.degrees(dR),
                 info["rms_mm"], info["loo_rms_mm"]))

    print()
    print("--- coplanar points (the trap) ---")
    print("  same 3 mm touch error, only the point geometry differs, 12 seeds:")
    stats = {}
    for label, plan in (("3D spread", 1.0), ("flat plate", 0.0)):
        dts, drs, spans = [], [], []
        for seed in range(12):
            r = np.random.default_rng(seed)
            P, Q = make(r, 10, noise_mm=3.0, planarity=plan)
            T, info = solve_point_registration(P, Q)
            dts.append(float(np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000))
            drs.append(math.degrees(float(np.linalg.norm(
                cv2.Rodrigues(T[:3, :3] @ T_true[:3, :3].T)[0]))))
            spans.append(info["span_ratio"])
        stats[label] = (float(np.median(spans)), float(np.median(dts)),
                        float(np.max(dts)), float(np.max(drs)))
        print("  %-10s : span %.3f | |dt| median %6.1f mm  worst %7.1f mm"
              " | |dR| worst %6.2f deg" % ((label,) + stats[label]))

    s3, m3, w3, _ = stats["3D spread"]
    sf, mf, wf, _ = stats["flat plate"]
    print()
    if sf < 0.05 and wf > 3.0 * max(w3, 1e-9):
        print("  -> the flat plate is unreliable, as expected: the median looks")
        print("     fine but the worst case is %.0fx worse. A small residual does"
              % (wf / max(w3, 1e-9)))
        print("     NOT prove the answer is right when the points are coplanar.")
    elif sf < 0.05:
        print("  -> the flat plate is flagged (span %.3f) but happened to solve"
              % sf)
        print("     well here. Do not rely on that - spread the points in 3D.")
    else:
        print("  !! the degeneracy detector did NOT fire on coplanar input")
        ok = False

    print()
    print("--- unaligned depth: colour pixel -> depth pixel ---")
    # a synthetic SLANTED plane seen by the depth camera, so the depth value
    # genuinely varies across the image and landing on the wrong pixel shows
    # up as an error (a flat plane would hide it - every pixel is the same)
    W, H = 640, 480
    K_c = np.array([[648.1, 0.0, 351.6],
                    [0.0, 655.6, 267.9],
                    [0.0, 0.0, 1.0]])
    K_d = np.array([[575.0, 0.0, 319.5],
                    [0.0, 575.0, 239.5],
                    [0.0, 0.0, 1.0]])

    n = np.array([0.18, -0.12, 1.0])
    n = n / np.linalg.norm(n)
    d_off = float(n @ np.array([0.0, 0.0, 0.70]))

    vv, uu = np.mgrid[0:H, 0:W]
    rx = (uu - K_d[0, 2]) / K_d[0, 0]
    ry = (vv - K_d[1, 2]) / K_d[1, 1]
    denom = n[0] * rx + n[1] * ry + n[2]
    tt = np.where(np.abs(denom) > 1e-9, d_off / denom, 0.0)
    depth_img = np.where((tt > 0.15) & (tt < 2.0), tt, 0.0).astype(np.float32)

    test_px = ((351.6, 267.9), (300.0, 220.0), (410.0, 300.0), (320.0, 210.0))
    cases = ((0.000, (0.0, 0.0, 0.0), "coincident"),
             (0.025, (0.0, 0.0, 0.0), "25 mm baseline"),
             (0.025, (0.03, 0.02, 0.05), "25 mm + tilt"))
    for baseline, rvec, label in cases:
        T_cd = np.eye(4)
        if any(rvec):
            T_cd[:3, :3] = cv2.Rodrigues(np.array(rvec))[0]
        T_cd[:3, 3] = [-baseline, 0.0, 0.0]      # depth cam sits at -x

        T_dc = inv(T_cd)
        O_d = T_dc[:3, 3]            # colour camera origin, IN THE DEPTH FRAME
        R_dc = T_dc[:3, :3]

        errs = []
        for uc, vc in test_px:
            # analytic truth: shoot the colour ray at the plane, seen from the
            # depth camera
            dir_c = np.array([(uc - K_c[0, 2]) / K_c[0, 0],
                              (vc - K_c[1, 2]) / K_c[1, 1], 1.0])
            dir_d = R_dc @ dir_c
            t_hit = (d_off - float(n @ O_d)) / float(n @ dir_d)
            P_d = O_d + t_hit * dir_d
            z_true = float((T_cd[:3, :3] @ P_d + T_cd[:3, 3])[2])

            zc, ud, vd, msg = map_pixel_to_depth(
                uc, vc, K_c, K_d, T_cd, depth_img,
                patch=3, dmin=0.15, dmax=2.0, guess=0.5)
            if zc is None:
                print("  %-16s pixel (%.0f, %.0f) FAILED: %s"
                      % (label, uc, vc, msg))
                ok = False
                continue
            errs.append(abs(zc - z_true) * 1000.0)

        if errs:
            worst = max(errs)
            print("  %-16s max Z error %6.2f mm over %d pixels  (want < 1 mm)"
                  % (label, worst, len(errs)))
            if worst > 1.0:
                ok = False

    print()
    print("--- too few points ---")
    P, Q = make(rng, 2)
    T, info = solve_point_registration(P, Q)
    print("  n=2 ->", info)
    if T is not None:
        ok = False

    print()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def parse_args():
    ap = argparse.ArgumentParser(
        description="eye-to-hand calibration by touching points (no marker)")
    ap.add_argument("--selftest", action="store_true",
                    help="run the synthetic self test and exit (no ROS needed)")

    ap.add_argument("--image", default="/camera/color/image_raw",
                    help="colour image topic")
    ap.add_argument("--depth", default="/camera/depth/image_raw",
                    help="depth image topic; aligned to colour or not, both work")
    ap.add_argument("--info", default="/camera/color/camera_info",
                    help="colour camera_info (used for the back-projection)")
    ap.add_argument("--depth-info", default="/camera/depth/camera_info",
                    help="depth camera_info; only needed when the depth image is "
                         "NOT aligned to colour")
    ap.add_argument("--depth-scale", type=float, default=0.001,
                    help="multiplier turning raw 16UC1 depth into metres (default 0.001 = mm)")
    ap.add_argument("--depth-min", type=float, default=0.15, help="metres")
    ap.add_argument("--depth-max", type=float, default=2.00, help="metres")
    ap.add_argument("--depth-patch", type=int, default=3,
                    help="half-width in pixels of the depth window used on the "
                         "unaligned path (default 3 -> a 7x7 window)")
    ap.add_argument("--depth-guess", type=float, default=0.5,
                    help="starting distance in metres for the unaligned pixel "
                         "search; only affects how fast it converges")

    ap.add_argument("--base-frame", default=BASE_FRAME)
    ap.add_argument("--tool-frame", default="tool_link")
    ap.add_argument("--tip-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z"),
                    help="pointer tip position expressed in the tool frame, metres")
    ap.add_argument("--child-frame", default=OPTICAL_FRAME,
                    help="name of the camera frame the result is expressed in")

    ap.add_argument("--hsv-lower", type=int, nargs=3, default=[40, 80, 80],
                    metavar=("H", "S", "V"), help="marker colour window (green by default)")
    ap.add_argument("--hsv-upper", type=int, nargs=3, default=[85, 255, 255],
                    metavar=("H", "S", "V"))
    ap.add_argument("--min-blob-area", type=float, default=20.0, help="pixels")
    ap.add_argument("--max-blob-area", type=float, default=20000.0, help="pixels")

    ap.add_argument("--samples", type=int, default=8,
                    help="how many touch points to record (default 8, use 8-12)")
    ap.add_argument("--min-separation", type=float, default=0.03,
                    help="reject a new point closer than this to an old one, metres")
    ap.add_argument("--debug-dir", default=None,
                    help="write an annotated image per capture here (handy headless)")
    ap.add_argument("--out", default="point_register_result.yaml")
    return ap.parse_args()


def save_debug(outdir, idx, color, mask, uv):
    os.makedirs(outdir, exist_ok=True)
    vis = color.copy()
    if uv is not None:
        cv2.drawMarker(vis, (int(round(uv[0])), int(round(uv[1]))),
                       (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    path = os.path.join(outdir, "capture_%02d.png" % idx)
    cv2.imwrite(path, vis)
    if mask is not None:
        cv2.imwrite(os.path.join(outdir, "mask_%02d.png" % idx), mask)
    return path


def main():
    args = parse_args()
    if args.selftest:
        return selftest()

    import rclpy
    import rclpy.executors

    print("configuration : eye-to-hand, POINT CONTACT (no fiducial marker)")
    print("colour        : %s" % args.image)
    print("depth         : %s" % args.depth)
    print("depth info    : %s" % args.depth_info)
    print("robot side    : %s -> %s + tip offset %s m"
          % (args.base_frame, args.tool_frame, list(args.tip_offset)))
    print("solving       : T_%s_%s" % (args.base_frame, args.child_frame))
    print("target        : %d touch points" % args.samples)

    rclpy.init()
    reg = PointRegistrar(args)

    # A single threaded executor in our own daemon thread.  MultiThreadedExecutor
    # keeps an internal thread pool, and tearing it down while spinning is what
    # produced "terminate called without an active exception" on exit.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(reg.node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    def cleanup():
        """stop spinning before anything is destroyed, then join."""
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        spin.join(timeout=2.0)
        try:
            reg.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    print("\nwaiting for the camera ...")
    for _ in range(150):
        if reg.ready():
            break
        time.sleep(0.1)
    else:
        print("!! camera not ready.  colour image:", reg.color is not None,
              " depth image:", reg.depth_m is not None,
              " colour intrinsics:", reg.K is not None)
        print("   is the camera driver running?")
        cleanup()
        return 1

    # the robot side is only needed to RECORD points.  Let the tool come up
    # camera-only so the marker, the depth and the HSV window can all be
    # checked before the arm is even powered.
    if reg.tool_tip_in_base()[0] is None:
        print("!! WARNING: no TF %s <- %s - the robot bringup is not running."
              % (args.base_frame, args.tool_frame))
        print("   Test mode (press t) still works, but nothing can be recorded.")

    print("encoding      : %s  ->  scaled by %g"
          % (reg.depth_enc, args.depth_scale))
    print("depth mode    : %s" % reg.depth_mode())
    for w in reg.alignment_warnings():
        print("!! WARNING: %s" % w)

    p_cam, p_base = [], []
    while len(p_cam) < args.samples:
        try:
            line = input("\n[%d/%d] put the pointer tip on a point, then Enter "
                         "(q to stop, t to test) > "
                         % (len(p_cam) + 1, args.samples))
        except EOFError:
            break
        if line.strip().lower() == "q":
            break

        test_only = line.strip().lower() == "t"

        tip, msg_tf = reg.tool_tip_in_base()
        if tip is None and not test_only:
            print("   FAIL tf   : %s" % msg_tf)
            continue

        p, msg_s, dbg = reg.sample()
        if args.debug_dir and reg.color is not None:
            print("               wrote", save_debug(args.debug_dir,
                                                     len(p_cam) + 1,
                                                     reg.color, dbg["mask"],
                                                     dbg["uv"]))
        if p is None:
            print("   FAIL      : %s" % msg_s)
            continue

        if test_only:
            print("   TEST only (not stored): %s" % msg_s)
            print("               cam  [%+.4f %+.4f %+.4f]" % (p[0], p[1], p[2]))
            if tip is None:
                print("               base unavailable (%s)" % msg_tf)
            else:
                print("               base [%+.4f %+.4f %+.4f]"
                      % (tip[0], tip[1], tip[2]))
            continue

        if p_base:
            dmin = min(float(np.linalg.norm(tip - q)) for q in p_base)
            if dmin < args.min_separation:
                print("   skipped   : only %.0f mm from an earlier point, "
                      "move further" % (dmin * 1000))
                continue

        p_cam.append(p)
        p_base.append(tip)
        print("   captured  : %s | %s" % (msg_s, msg_tf))
        print("               cam  [%+.4f %+.4f %+.4f]" % (p[0], p[1], p[2]))
        print("               base [%+.4f %+.4f %+.4f]" % (tip[0], tip[1], tip[2]))

    print("\ncollected %d points, solving ..." % len(p_cam))
    T, info = solve_point_registration(p_cam, p_base)

    if T is None:
        print("!! solve failed: %s" % info)
        cleanup()
        return 1

    t = T[:3, 3]
    rpy = mat_to_rpy(T[:3, :3])
    quat = mat_to_quat(T[:3, :3])

    print("\n" + "=" * 66)
    print("RESULT  -  camera pose in the %s frame" % args.base_frame)
    print("=" * 66)
    print("frame             : %s -> %s" % (args.base_frame, args.child_frame))
    print("points used       : %d" % info["n"])
    print("translation xyz   : [%.4f, %.4f, %.4f] m" % (t[0], t[1], t[2]))
    print("rotation rpy      : [%.4f, %.4f, %.4f] rad (%.2f, %.2f, %.2f deg)"
          % (rpy[0], rpy[1], rpy[2],
             math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2])))
    print("quaternion xyzw   : [%.5f, %.5f, %.5f, %.5f]"
          % (quat[0], quat[1], quat[2], quat[3]))
    print("det(R)            : %.6f  (must be +1.000000)" % info["det"])

    print("\nresidual - how well one rigid transform explains every point")
    for i, e in enumerate(info["residual_mm"], 1):
        print("  point %2d : %6.2f mm" % (i, e))
    print("  fit rms  : %6.2f mm   (want < 5 mm)" % info["rms_mm"])
    print("  fit max  : %6.2f mm" % info["max_mm"])
    if info["loo_rms_mm"] is not None:
        print("  leave-one-out rms : %6.2f mm   <- the honest number, measured "
              "on points kept out of the fit" % info["loo_rms_mm"])

    print("\ngeometry check")
    print("  touch point spread (cam) : [%.3f %.3f %.3f] m  (singular values)"
          % tuple(info["singular_values"]))
    print("  3D span ratio            : %.3f   (want > 0.05)" % info["span_ratio"])
    print("  closest pair of points   : %.0f mm" % (info["min_pair_dist_m"] * 1000))

    verdict = []
    if info["span_ratio"] < 0.05:
        verdict.append("points are nearly COPLANAR - the depth direction is "
                       "barely constrained. Touch points at different heights.")
    if info["min_pair_dist_m"] < 0.05:
        verdict.append("some points are almost on top of each other - spread "
                       "them out.")
    if info["loo_rms_mm"] is not None and info["loo_rms_mm"] > 10.0:
        verdict.append("leave-one-out error is large - check depth alignment, "
                       "the tip offset, and whether the marker stays centred "
                       "on the tip.")
    if abs(info["det"] - 1.0) > 1e-6:
        verdict.append("det(R) is not +1, the solve degenerated.")
    if verdict:
        print("\n!! " + "\n!! ".join(verdict))
    else:
        print("\n-> geometry and residuals look good.")

    # ready-to-paste TF publisher
    print("\npublish it into TF:")
    print("  ros2 run tf2_ros static_transform_publisher \\")
    print("    --x %.5f --y %.5f --z %.5f \\" % (t[0], t[1], t[2]))
    print("    --qx %.6f --qy %.6f --qz %.6f --qw %.6f \\"
          % (quat[0], quat[1], quat[2], quat[3]))
    print("    --frame-id %s --child-frame-id %s"
          % (args.base_frame, args.child_frame))

    with open(args.out, "w") as f:
        f.write("# eye-to-hand calibration result, POINT CONTACT method\n")
        f.write("# camera pose in the %s frame\n" % args.base_frame)
        f.write("method: point_contact_kabsch\n")
        f.write("base_frame: %s\n" % args.base_frame)
        f.write("child_frame: %s\n" % args.child_frame)
        f.write("points_used: %d\n" % info["n"])
        f.write("translation_xyz_m: [%f, %f, %f]\n" % (t[0], t[1], t[2]))
        f.write("quaternion_xyzw: [%f, %f, %f, %f]\n"
                % (quat[0], quat[1], quat[2], quat[3]))
        f.write("rpy_rad: [%f, %f, %f]\n" % (rpy[0], rpy[1], rpy[2]))
        f.write("fit_rms_mm: %f\n" % info["rms_mm"])
        f.write("fit_max_mm: %f\n" % info["max_mm"])
        if info["loo_rms_mm"] is not None:
            f.write("leave_one_out_rms_mm: %f\n" % info["loo_rms_mm"])
        f.write("span_ratio: %f\n" % info["span_ratio"])
        f.write("det_R: %f\n" % info["det"])
        f.write("tip_offset_m: [%f, %f, %f]\n" % tuple(args.tip_offset))
        f.write("depth_topic_scale: %g\n" % args.depth_scale)
    print("\nwritten: %s" % args.out)

    cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
