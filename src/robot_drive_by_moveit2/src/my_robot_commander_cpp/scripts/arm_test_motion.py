#!/usr/bin/env python3
"""
arm_test_motion - safe first-motion test for the REAL arm.

Sends one small, slow joint-space trajectory straight to arm_controller
(NOT through MoveIt/move_group), so a first motion test does not depend on
planning, IK or collision checks.

Motion: joint2 only, from current pose to +20 deg and back to 0.
Adjust joints/angles below for your own tests.

Usage:
  # terminal 1: ros2 launch my_robot_bringup real_arm.launch.xml
  # terminal 2:
  ros2 run my_robot_commander_cpp arm_test_motion.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]


def make_point(rad_list, t):
    pt = JointTrajectoryPoint()
    pt.positions = rad_list
    pt.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
    return pt


class ArmTestMotion(Node):
    def __init__(self):
        super().__init__("arm_test_motion")
        self.pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)

    def send_sweep(self):
        deg20 = math.radians(20.0)
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        msg.points = [
            make_point([0.0, 0.0, 0.0, 0.0, 0.0], 1.0),      # stay
            make_point([0.0, deg20, 0.0, 0.0, 0.0], 4.0),    # joint2 +20 deg
            make_point([0.0, 0.0, 0.0, 0.0, 0.0], 8.0),      # back to 0
        ]
        self.get_logger().info("sending joint2 sweep (+20deg then back)...")
        self.pub.publish(msg)

    def spin_then_exit(self):
        # publish a few times to survive any transient startup
        for _ in range(3):
            self.send_sweep()
            time.sleep(0.5)
        self.get_logger().info("trajectory sent, check the real arm")


def main():
    rclpy.init()
    node = ArmTestMotion()
    # small delay so controller is up
    time.sleep(2.0)
    node.spin_then_exit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
