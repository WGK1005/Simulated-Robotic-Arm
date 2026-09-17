"""Verification of the AprilTag detection + pose pipeline in tag_pose.py.

Renders a synthetic AprilTag of a known pixel size, puts it in a synthetic
image, and checks that the pose recovered by TagDetector matches the pose
that must have produced that image.

If a tag of real size S metres occupies N pixels and the camera has focal
length fx, the tag must be at distance  Z = fx * S / N.

Run:  python3 test_tag_pose.py
"""

import math

import numpy as np
import cv2

from tag_pose import TagDetector


def render_scene(tag_px, image_w=640, image_h=480, tag_id=0, centre=None):
    """White image with the tag centred at `centre` (default image centre)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "generateImageMarker"):      # OpenCV >= 4.7
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px)
    else:                                              # OpenCV < 4.7
        marker = cv2.aruco.drawMarker(dictionary, tag_id, tag_px)

    img = np.full((image_h, image_w), 255, dtype=np.uint8)
    cx, cy = centre if centre else (image_w // 2, image_h // 2)
    x0 = int(round(cx - tag_px / 2.0))
    y0 = int(round(cy - tag_px / 2.0))
    img[y0:y0 + tag_px, x0:x0 + tag_px] = marker
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def main():
    fx = fy = 648.108
    cx, cy = 351.635, 267.930
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.zeros((5, 1))

    tag_size = 0.065                     # 65 mm AprilTag
    detector = TagDetector(tag_size, tag_id=0)
    print("pnp flag  :", detector.pnp_flag,
          "(IPPE_SQUARE =", getattr(cv2, "SOLVEPNP_IPPE_SQUARE", None), ")")
    print("detector  :", "ArucoDetector class" if detector.detector else
          "detectMarkers (OpenCV < 4.7)")
    print()

    ok_all = True
    # A fronto-parallel tag seen by the camera comes back as a ~180 deg
    # rotation about x, because the fiducial frame has +z pointing out of the
    # tag towards the camera while the camera's +z points away from it.
    R_frontal = np.diag([1.0, -1.0, -1.0])

    print("%-10s %-12s %-12s %-10s %-10s" %
          ("tag_px", "expected Z", "measured Z", "rel err", "R err deg"))
    print("-" * 64)
    for tag_px in (40, 80, 120, 200, 320):
        img = render_scene(tag_px, tag_id=0, centre=(cx, cy))
        T, msg = detector.pose(img, K, D)
        if T is None:
            print("%-10d FAILED: %s" % (tag_px, msg))
            ok_all = False
            continue
        expected = fx * tag_size / tag_px
        measured = T[2, 3]
        rel = abs(measured - expected) / expected
        # how far the recovered rotation is from the expected fronto-parallel one
        rerr = math.degrees(float(np.linalg.norm(
            cv2.Rodrigues(T[:3, :3] @ R_frontal)[0])))
        # Corner positions are quantised to whole pixels, so a tag that only
        # spans a few dozen pixels has an inherent relative error of a couple
        # of percent. Accuracy improves roughly as 1/tag_px:
        #   40 px -> 2.6 %, 200 px -> 0.5 %
        # Practical rule: keep the tag at least ~80 px wide on the sensor.
        strict = tag_px >= 80
        good = rerr < 1.0 and (rel < 0.02 if strict else True)
        if not good:
            ok_all = False
        print("%-10d %-12.4f %-12.4f %-9.2f%% %-10.3f %s"
              % (tag_px, expected, measured, rel * 100, rerr,
                 "OK" if good else "BAD"))

    # tag placed exactly on the principal point -> no lateral offset
    img = render_scene(200, tag_id=0, centre=(cx, cy))
    T, _ = detector.pose(img, K, D)
    lateral = math.hypot(T[0, 3], T[1, 3])
    print()
    print("tag on the principal point -> lateral offset should be ~0: %.4f m"
          % lateral)
    if lateral > 0.005:
        ok_all = False

    # a tag id that is not in the image must be reported, not silently wrong
    detector2 = TagDetector(tag_size, tag_id=7)
    _, msg = detector2.pose(img, K, D)
    print("asking for a missing id -> '%s'" % msg)
    if "not visible" not in msg:
        ok_all = False

    print()
    print("RESULT:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
