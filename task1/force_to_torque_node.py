#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node

from .planar_arm import Planar3DoFArm


class ForceToTorqueNode(Node):
    """
    Simple ROS2 node that computes joint torques from an end-effector force:

        tau = J(q)^T * F

    For now, q and F are fixed test values.
    Later we can extend this to subscribers/services.
    """

    def __init__(self):
        super().__init__("force_to_torque_node")

        # Create arm model
        self.arm = Planar3DoFArm()

        # Example joint configuration (in radians)
        self.q = [0.0, math.pi / 4.0, -math.pi / 6.0]

        # Example end-effector wrench: [Fx, Fy, tau_z]
        # You can tweak these numbers later.
        self.F = np.array([5.0, 2.0, 0.5])

        # Timer to recompute and print torques
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("ForceToTorqueNode started.")

    def timer_callback(self):
        J = self.arm.jacobian(self.q)          # 3x3 Jacobian
        tau = J.T @ self.F                     # tau = J^T * F

        # Log nicely
        self.get_logger().info(
            f"q = {self.q}\n"
            f"F = {self.F}\n"
            f"J =\n{J}\n"
            f"tau (joint torques) = {tau}\n"
            "------------------------"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForceToTorqueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

