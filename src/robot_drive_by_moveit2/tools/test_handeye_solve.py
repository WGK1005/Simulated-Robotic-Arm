"""Verification of the AX=ZB solve used in hand_eye_calibrate.py.

Eye-to-hand: camera FIXED in the base, AprilTag board rigidly on the tool.

    A_i = T_base_tool   (robot forward kinematics)
    B_i = T_cam_tag     (AprilTag detection)
    X   = T_tool_tag    (board mount, unknown constant)
    Z   = T_base_cam    (what we want, unknown constant)
    relation:  A_i X = Z B_i

This test builds synthetic data from a known (X, Z), runs the same solve()
logic that hand_eye_calibrate.py uses, and checks that Z comes back.

Run:  python3 test_handeye_solve.py
"""
import math

import numpy as np
import cv2


def rand_T(rng, tmax=0.4):
    """random rigid transform with a decent rotation (needed for excitation)"""
    T = np.eye(4)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    T[:3, :3] = cv2.Rodrigues(axis * rng.uniform(0.3, math.pi))[0]
    T[:3, 3] = rng.uniform(-tmax, tmax, 3)
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    o = np.eye(4)
    o[:3, :3] = R.T
    o[:3, 3] = -R.T @ t
    return o


def solve(samples):
    """same logic as hand_eye_calibrate.py::solve"""
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for A, B in samples:
        A_inv = inv(A)                      # eye-to-hand: invert the robot poses
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
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, None, None, m)
        except Exception:
            continue
        Z = np.eye(4)
        Z[:3, :3] = R
        Z[:3, 3] = t.flatten()
        results.append((m, Z))
    if not results:
        return None, "all methods failed"

    best = None
    for m, Z in results:
        Xs = [inv(A) @ Z @ B for A, B in samples]
        t_spread = np.std([X[:3, 3] for X in Xs], axis=0).max()
        R_ref = Xs[0][:3, :3]
        rot_spread = max(float(np.linalg.norm(cv2.Rodrigues(X[:3, :3] @ R_ref.T)[0]))
                         for X in Xs)
        score = t_spread + 0.1 * rot_spread
        if best is None or score < best[0]:
            best = (score, m, Z, t_spread, rot_spread)
    _, method, Z, t_spread, rot_spread = best
    return Z, (method, t_spread, rot_spread)


def build_dataset(rng, n, Z_true, X_true, noise_mm=0.0, noise_deg=0.0):
    samples = []
    Zinv = inv(Z_true)
    for _ in range(n):
        A = rand_T(rng)                     # T_base_tool
        B = Zinv @ A @ X_true               # T_cam_tag
        if noise_mm or noise_deg:
            B = B.copy()
            B[:3, 3] += rng.normal(0, noise_mm / 1000.0, 3)
            B[:3, :3] = B[:3, :3] @ cv2.Rodrigues(
                rng.normal(0, math.radians(noise_deg), 3))[0]
        samples.append((A, B))
    return samples


def main():
    rng = np.random.default_rng(7)

    Z_true = np.eye(4)
    Z_true[:3, :3] = cv2.Rodrigues(np.array([1.2, 0.3, 2.0]))[0]
    Z_true[:3, 3] = [0.35, -0.10, 0.60]     # camera 35cm fwd, 10cm left, 60cm up

    X_true = np.eye(4)
    X_true[:3, :3] = cv2.Rodrigues(np.array([0.1, 0.0, 0.5]))[0]
    X_true[:3, 3] = [0.0, 0.0, 0.13]        # board 13cm past the tool frame

    print("ground-truth camera-in-base translation:", Z_true[:3, 3])
    print()

    ok = True
    print("--- noise free ---")
    for n in (5, 8, 15, 30):
        Z, info = solve(build_dataset(rng, n, Z_true, X_true))
        if Z is None:
            print("n=%2d  FAILED: %s" % (n, info))
            ok = False
            continue
        method, t_spread, rot_spread = info
        dt = float(np.linalg.norm(Z[:3, 3] - Z_true[:3, 3]))
        dR = float(np.linalg.norm(cv2.Rodrigues(Z[:3, :3] @ Z_true[:3, :3].T)[0]))
        status = "OK " if (dt < 1e-6 and dR < 1e-6) else "BAD"
        if status == "BAD":
            ok = False
        print("n=%2d  %s  |dt|=%.2e m  |dR|=%.2e rad   (T_tool_tag spread "
              "%.2e m / %.2e rad)" % (n, status, dt, dR, t_spread, rot_spread))

    print()
    print("--- with detection noise (20 samples) ---")
    for mm, deg in ((1.0, 0.2), (2.0, 0.5), (5.0, 1.0)):
        Z, info = solve(build_dataset(rng, 20, Z_true, X_true, mm, deg))
        dt = float(np.linalg.norm(Z[:3, 3] - Z_true[:3, 3]))
        dR = float(np.linalg.norm(cv2.Rodrigues(Z[:3, :3] @ Z_true[:3, :3].T)[0]))
        print("noise %4.1f mm / %.1f deg -> |dt| = %6.1f mm   |dR| = %5.2f deg"
              % (mm, deg, dt * 1000, math.degrees(dR)))

    print()
    print("--- wrong convention (for contrast) ---")
    samples = build_dataset(rng, 15, Z_true, X_true)
    Rg = [A[:3, :3] for A, _ in samples]          # NOT inverted - wrong for eye-to-hand
    tg = [A[:3, 3].reshape(3, 1) for A, _ in samples]
    Rt = [B[:3, :3] for _, B in samples]
    tt = [B[:3, 3].reshape(3, 1) for _, B in samples]
    R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_PARK)
    dt = float(np.linalg.norm(t.flatten() - Z_true[:3, 3]))
    print("passing non-inverted poses -> |dt| = %.3f m  (garbage, as expected)" % dt)

    print()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
