#!/usr/bin/env python3
"""
Minimal MuJoCo demo for the 5-DOF arm + parallel gripper.

Usage
-----
  python3 demo.py                 # open the interactive viewer
  python3 demo.py --headless      # no GUI, just step the model and report
  python3 demo.py --model /path/to/my_robot.xml
  python3 demo.py --keyframe ready

The viewer needs a display. On WSLg (Windows 11) it normally works out of
the box; over plain SSH use --headless, or forward X11 / use VNC.

Install MuJoCo first:
  pip install mujoco numpy
"""

import argparse
import os
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_XML = os.path.join(HERE, "my_robot.xml")

# qpos / ctrl indices for the 6 actuators and 7 dofs
IDX_J1, IDX_J2, IDX_J3, IDX_J4, IDX_J5 = 0, 1, 2, 3, 4
IDX_GRIP = 5


def describe(m):
    print("nq (dofs)  =", m.nq)
    print("nv         =", m.nv)
    print("nu (actr)  =", m.nu)
    print("timestep   =", m.opt.timestep)
    print("joints:")
    for i in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        lo, hi = m.jnt_range[i]
        kind = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(
            int(m.jnt_type[i]), "?")
        print("  %-28s %-6s range [%+.3f, %+.3f]" % (name, kind, lo, hi))
    total = sum(m.body_mass)
    print("total mass = %.3f kg" % total)
    for i in range(m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
        if m.body_mass[i] > 0:
            print("  body %-28s %.4f kg" % (name, m.body_mass[i]))


def control(t, d):
    """Simple motion so you can see the model is alive.

    t is SIMULATED time (d.time), not wall-clock time: a headless run can be
    hundreds of times faster than real time, so driving the control from wall
    time would freeze the motion.

    - joint2 sweeps +-0.4 rad
    - gripper opens/closes in sync
    """
    d.ctrl[IDX_J2] = 0.4 * np.sin(2 * np.pi * 0.2 * t)
    d.ctrl[IDX_GRIP] = 0.02 + 0.02 * np.sin(2 * np.pi * 0.2 * t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=DEFAULT_XML, help="path to the MJCF file")
    ap.add_argument("--headless", action="store_true",
                    help="no viewer - just step the model (works over SSH)")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="headless run duration in seconds")
    ap.add_argument("--keyframe", default="home", help="keyframe to start from")
    args = ap.parse_args()

    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        raise SystemExit("model not found: %s" % model_path)

    print("loading:", model_path)
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    describe(m)

    # start pose
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(m, d, key_id)
        print("reset to keyframe '%s'" % args.keyframe)
    else:
        print("keyframe '%s' not found, starting from qpos=0" % args.keyframe)

    if args.headless:
        # run a fixed amount of SIMULATED time, and drive the control from it
        n_steps = int(round(args.seconds / m.opt.timestep))
        log_every = max(1, n_steps // 10)
        print("running %.2f s of simulated time (%d steps)..." % (args.seconds, n_steps))
        print("  %8s %10s %10s %12s %6s" % ("sim_t", "ctrl[j2]", "qpos[j2]", "actf[j2]", "ncon"))
        for i in range(n_steps):
            control(d.time, d)
            mujoco.mj_step(m, d)
            if i % log_every == 0:
                print("  %8.2f %+10.3f %+10.4f %+12.3f %6d"
                      % (d.time, d.ctrl[IDX_J2], d.qpos[IDX_J2],
                         d.actuator_force[IDX_J2], d.ncon))
        print("sim time = %.2f s, steps = %d" % (d.time, n_steps))
        print("final qpos =", np.round(d.qpos, 4))
        return

    # NOTE: import as "from mujoco import viewer" - a plain "import mujoco.viewer"
    # inside a function would make "mujoco" a local name and break the calls above.
    try:
        from mujoco import viewer as mj_viewer
    except Exception as exc:
        raise SystemExit(
            "cannot load the viewer (%s).\n"
            "Use --headless, or set up a display:\n"
            "  WSLg  -> should work out of the box (echo $DISPLAY)\n"
            "  X11   -> install VcXsrv on Windows and export DISPLAY=:0\n"
            % exc)

    # pace the sim to roughly real time, control driven by simulated time
    next_tick = time.time()
    with mj_viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            control(d.time, d)
            mujoco.mj_step(m, d)
            viewer.sync()
            next_tick += m.opt.timestep
            delay = next_tick - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.time()   # we fell behind, resync


if __name__ == "__main__":
    main()
