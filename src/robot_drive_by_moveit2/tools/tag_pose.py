"""AprilTag (36h11) detection + single-tag pose estimation.

Deliberately free of ROS imports so it can be unit-tested and reused.

Used by hand_eye_calibrate.py; verified by test_tag_pose.py.
"""

import numpy as np
import cv2


class TagDetector:
    """Detects one AprilTag family 36h11 and returns its pose in the camera.

    Works on both OpenCV < 4.7 (DetectorParameters_create,
    cv2.aruco.detectMarkers) and >= 4.7 (DetectorParameters,
    cv2.aruco.ArucoDetector).
    """

    def __init__(self, tag_size_m, tag_id=0, dictionary_id=None):
        if dictionary_id is None:
            dictionary_id = cv2.aruco.DICT_APRILTAG_36h11
        self.tag_size = float(tag_size_m)
        self.tag_id = int(tag_id)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        try:
            self.params = cv2.aruco.DetectorParameters()
        except Exception:
            self.params = cv2.aruco.DetectorParameters_create()

        self.detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        # best flag for a square planar target; IPPE_SQUARE needs OpenCV >= 4.5
        self.pnp_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)

        # NOTE on the tag frame.
        #
        # cv2.SOLVEPNP_IPPE_SQUARE requires this exact corner order:
        #     (-s/2, +s/2), (+s/2, +s/2), (+s/2, -s/2), (-s/2, -s/2)
        # and aruco returns the image corners in the matching order
        # (top-left, top-right, bottom-right, bottom-left). Reordering the
        # model points silently breaks IPPE_SQUARE (it returns Z=0).
        #
        # With this convention the recovered frame is the usual fiducial one:
        # +x right, +y up in the image, +z out of the tag TOWARDS the camera.
        # A fronto-parallel tag therefore comes back rotated ~180 deg about x
        # relative to the camera - that is expected, and harmless: the tag
        # frame definition is arbitrary, and the hand-eye solve absorbs any
        # fixed re-definition into X = T_tool_tag, leaving Z = T_base_cam
        # unchanged. See test_tag_pose.py.
        h = self.tag_size / 2.0
        self.object_points = np.array([[-h, h, 0.0],
                                       [h, h, 0.0],
                                       [h, -h, 0.0],
                                       [-h, -h, 0.0]], dtype=np.float64)

    # ------------------------------------------------------------------ detect
    def detect(self, gray):
        """Return (corners, ids) for every tag found in the gray image."""
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.params)
        return corners, ids

    # ------------------------------------------------------------------- pose
    def pose(self, image_bgr, K, D):
        """Pose of the wanted tag in the camera frame.

        Returns (T_cam_tag 4x4 or None, message string).
        """
        if image_bgr is None:
            return None, "no image"
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)
        if ids is None or len(ids) == 0:
            return None, "no tag visible"

        idx = np.where(ids.flatten() == self.tag_id)[0]
        if len(idx) == 0:
            return None, "tag id %d not visible (saw %s)" % (
                self.tag_id, sorted(int(i) for i in ids.flatten()))

        img_pts = corners[idx[0]][0].astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(self.object_points, img_pts, K, D,
                                      flags=self.pnp_flag)
        if not ok:
            return None, "solvePnP failed"

        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.flatten()

        area = cv2.contourArea(img_pts.astype(np.float32))
        return T, "ok (tag area %.0f px)" % area
