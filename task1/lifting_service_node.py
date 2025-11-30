#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .planar_arm import Planar3DoFArm


class LiftingServiceNode(Node):
    """
    Node that simulates starting/stopping cooperative lifting behavior.
    """

    def __init__(self):
        super().__init__('lifting_service_node')

        self.arm = Planar3DoFArm()
        self.q = [0.0, math.pi / 4.0, -math.pi / 6.0]

        # Example wrench
        self.F = np.array([5.0, 2.0, 0.5])

        # Lifting state
        self.lifting = False

        # Simple lifting simulation
        self.object_height = 0.0      # meters
        self.target_height = 0.20     # lift until 20 cm
        self.height_gain = 0.001      # proportional factor

        # Create services
        self.start_srv = self.create_service(
            Trigger,
            'start_lifting',
            self.start_lifting_cb
        )
        self.stop_srv = self.create_service(
            Trigger,
            'stop_lifting',
            self.stop_lifting_cb
        )

        # Timer
        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info('LiftingServiceNode started. Waiting for service calls.')

    def start_lifting_cb(self, request, response):
        self.lifting = True
        response.success = True
        response.message = 'Lifting started.'
        self.get_logger().info('Received /start_lifting request -> lifting = True')
        return response

    def stop_lifting_cb(self, request, response):
        self.lifting = False
        response.success = True
        response.message = 'Lifting stopped.'
        self.get_logger().info('Received /stop_lifting request -> lifting = False')
        return response

    def timer_cb(self):
        if not self.lifting:
            return

        # Compute tau = J^T * F
        J = self.arm.jacobian(self.q)
        tau = J.T @ self.F

        # Increase object height based on force
        fy = self.F[1]
        self.object_height += self.height_gain * fy

        # Stop automatically when done
        if self.object_height >= self.target_height:
            self.object_height = self.target_height
            self.lifting = False
            self.get_logger().info(
                f"[LIFTING DONE] Reached target height = {self.object_height:.3f} m. Stopping lifting."
            )
        else:
            self.get_logger().info(
                f"[LIFTING ACTIVE]\n"
                f"q = {self.q}\n"
                f"F = {self.F}\n"
                f"tau (joint torques) = {tau}\n"
                f"object_height = {self.object_height:.3f} m\n"
                "------------------------"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LiftingServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

