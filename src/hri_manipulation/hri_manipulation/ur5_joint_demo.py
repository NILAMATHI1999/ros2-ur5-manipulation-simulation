#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class UR5JointDemo(Node):
    def __init__(self):
        super().__init__("ur5_joint_demo")

        self.joint_names: List[str] = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.t = 0.0
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info("UR5JointDemo started, publishing /joint_states")

    def timer_cb(self):
        q0 = 0.5 * math.sin(self.t)
        q1 = -0.8 + 0.3 * math.sin(self.t * 0.7)
        q2 = 1.2 + 0.2 * math.sin(self.t * 1.1)
        q3 = 0.0
        q4 = 0.0
        q5 = 0.0

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [q0, q1, q2, q3, q4, q5]

        self.pub.publish(js)
        self.t += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = UR5JointDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
