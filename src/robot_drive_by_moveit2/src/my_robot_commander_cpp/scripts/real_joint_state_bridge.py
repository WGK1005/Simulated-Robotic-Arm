#!/usr/bin/env python3
"""
Real joint-state bridge for the 5-DOF arm (ZP25S serial bus servos).

ROUTE A - read-only:
  polls the 5 servos (#<ID>PRAD! -> #<ID>Pxxxx!) and publishes their
  angles (converted to URDF radians) on /joint_states, so the RViz model
  follows the real robot.

Calibration (measured on the real arm):
  * servo zero position   P = 1500  <->  URDF joint angle 0 (arm vertical)
  * scale                 900 P-steps = 120 deg  ->  1 P-step = 0.1333 deg
  * conversion            rad = (P - 1500) * 0.1333 * (pi/180) * direction

Usage (after colcon build):
  ros2 run my_robot_commander_cpp real_joint_state_bridge.py
  ros2 run my_robot_commander_cpp real_joint_state_bridge.py --demo
"""

import math
import re
import sys
import time

import rclpy
from rclpy.node import Node as RosNode
from sensor_msgs.msg import JointState

PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_ZERO_P = 1500             # P read when joint angle = 0 (arm vertical)
DEG_PER_PSTEP = 120.0 / 900.0   # measured: +900 steps == +120 deg
DEG2RAD = math.pi / 180.0

# (joint name, servo ID, direction)
# direction +1: larger P rotates the joint the URDF-positive way.
# direction -1: flip this entry if the model moves opposite the real arm.
JOINTS = [
    ("joint1", "001", 1.0),
    ("joint2", "002", 1.0),
    ("joint3", "003", 1.0),
    ("joint4", "004", 1.0),
    ("joint5", "005", 1.0),
]

PUBLISH_HZ = 10.0
READ_TIMEOUT_S = 0.05


def p_to_rad(p, direction):
    deg = (p - SERVO_ZERO_P) * DEG_PER_PSTEP * direction
    return deg * DEG2RAD


class RealJointStateBridge(RosNode):
    def __init__(self):
        super().__init__("real_joint_state_bridge")
        import serial  # python3-serial must be installed

        self.ser = serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT_S)
        self.ser.reset_input_buffer()
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.last_pos = {}    # joint name -> last known P value
        self.read_fail = {}   # joint name -> consecutive read failures

    # ------------------------------------------------------------------ servo
    def read_pos(self, sid):
        """Send #<ID>PRAD! and parse the '#<ID>Pxxxx!' reply. int or None."""
        try:
            self.ser.reset_input_buffer()
            self.ser.write(f"#{sid}PRAD!".encode("utf-8"))
            self.ser.flush()
            line = self.ser.readline()
            if not line:
                return None
            m = re.search(rb"P(\d{3,5})", line)
            if m:
                return int(m.group(1))
        except Exception as exc:   # never let one bad read kill the node
            self.get_logger().warn(f"read {sid} failed: {exc}")
        return None

    def write_pos(self, sid, pos, duration_ms=400):
        """Send '#<ID>PxxxxTxxxx!' - only used by the --demo sweep."""
        cmd = f"#{sid}P{int(pos):04d}T{duration_ms:04d}!"
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()
        time.sleep(duration_ms / 1000.0 + 0.1)

    # ------------------------------------------------------------------ loop
    def run(self):
        if "--demo" in sys.argv:
            self.demo_sweep()

        period = 1.0 / PUBLISH_HZ
        while rclpy.ok():
            names = []
            positions = []
            for name, sid, direction in JOINTS:
                p = self.read_pos(sid)
                if p is None:
                    self.read_fail[name] = self.read_fail.get(name, 0) + 1
                    if self.read_fail[name] == 10:
                        self.get_logger().warn(
                            f"servo {sid} ({name}) not responding")
                    p = self.last_pos.get(name)   # reuse last known value
                    continue
                self.read_fail[name] = 0
                self.last_pos[name] = p
                names.append(name)
                positions.append(round(p_to_rad(p, direction), 6))

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            self.pub.publish(msg)

            time.sleep(period)

    # ------------------------------------------------------------- demo sweep
    def demo_sweep(self):
        """Prove direction mapping in RViz: centre, +40deg, centre, -40deg."""
        self.get_logger().info("demo sweep on all joints")
        for name, sid, direction in JOINTS:
            self.get_logger().info(f"demo sweep {name} (ID {sid})")
            self.write_pos(sid, SERVO_ZERO_P)
            self.write_pos(sid, SERVO_ZERO_P + int(300 * direction))
            self.write_pos(sid, SERVO_ZERO_P)
            self.write_pos(sid, SERVO_ZERO_P - int(300 * direction))
            self.write_pos(sid, SERVO_ZERO_P)


def main():
    rclpy.init()
    node = RealJointStateBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
