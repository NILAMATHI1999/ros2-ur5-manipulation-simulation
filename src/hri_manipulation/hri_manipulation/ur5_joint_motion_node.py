#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR5JointMotionNode(Node):
    """
    Simple node that publishes joint_states for the UR5
    so the robot moves in RViz without using the GUI sliders.
    """

    def __init__(self):
        super().__init__("ur5_joint_motion_node")

        # Publish on the same topic as joint_state_publisher_gui
        self.publisher = self.create_publisher(JointState, "joint_states", 10)

        # Timer: 50 ms -> 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.t = 0.0
        self.get_logger().info("UR5JointMotionNode started (publishing /joint_states).")

    def timer_cb(self):
        self.t += self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = UR5_JOINT_NAMES

        # Simple sinus motion around a neutral pose
        amp = 0.8    # amplitude
        w = 0.5      # speed

        shoulder_pan   = amp * math.sin(w * self.t)
        shoulder_lift  = -0.5 + 0.2 * math.sin(w * self.t + 0.5)
        elbow          = amp * math.sin(w * self.t + 1.0)
        wrist_1        = 0.5 * math.sin(w * self.t + 1.5)
        wrist_2        = 0.3 * math.sin(w * self.t + 2.0)
        wrist_3        = 0.3 * math.sin(w * self.t + 2.5)

        msg.position = [
            shoulder_pan,
            shoulder_lift,
            elbow,
            wrist_1,
            wrist_2,
            wrist_3,
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR5JointMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
