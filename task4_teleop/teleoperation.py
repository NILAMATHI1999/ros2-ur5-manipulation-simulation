import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import time


class TeleoperationNode(Node):
    """
    SIMULATED teleoperation node:

    - Generates joystick-like movement (sinusoidal twist)
    - Subscribes to 'force_mapped'
    - Prints vibration intensity based on force_mapped
    - Publishes twist to 'cmd_vel'
    - Publishes gripper width to 'gripper_width'
    """

    def __init__(self):
        super().__init__('teleoperation_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.grip_pub = self.create_publisher(Float32, 'gripper_width', 10)

        # Subscriber
        self.sub_force = self.create_subscription(
            Float32,
            'force_mapped',
            self.force_callback,
            10
        )

        # Timer for simulated movement
        self.timer = self.create_timer(0.1, self.joystick_sim)

        self.latest_force = 0.0
        self.get_logger().info("Teleoperation node started (SIMULATED joystick + haptics).")

    def joystick_sim(self):
        """Simulate joystick movement using time-based signals."""

        t = time.time()

        twist = Twist()
        twist.linear.x = 0.2 * math.sin(t)
        twist.angular.z = 0.3 * math.cos(t)

        # example gripper width: open/close rhythmically
        grip = Float32()
        grip.data = 0.05 + 0.03 * math.sin(t)

        self.cmd_pub.publish(twist)
        self.grip_pub.publish(grip)

        self.get_logger().info(
            f"Sim Twist: vx={twist.linear.x:.3f}, wz={twist.angular.z:.3f}, Grip={grip.data:.3f}"
        )

        # display haptic vibration
        vibration = self.latest_force  # already between 0 and 1
        self.get_logger().info(f"Haptic vibration intensity: {vibration:.2f}")

    def force_callback(self, msg: Float32):
        """Receive mapped force 0–1."""
        self.latest_force = msg.data
        self.get_logger().info(f"Received mapped force={self.latest_force:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
