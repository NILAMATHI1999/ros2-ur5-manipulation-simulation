import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import time


class FSRSensor(Node):
    """
    Publishes a simulated force value: 0 → max_force on topic 'pub_force'.
    """

    def __init__(self):
        super().__init__('fsr_sensor')

        self.pub = self.create_publisher(Float32, 'pub_force', 10)

        # Simulated max force (you can change if you want)
        self.max_force = 20.0

        # 10 Hz timer
        self.timer = self.create_timer(0.1, self.publish_force)

        self.get_logger().info("FSR Sensor Node Started (SIMULATED)")

    def publish_force(self):
        # Example: oscillating simulated force between 0 and max_force
        t = time.time()
        force = (self.max_force / 2.0) * (1.0 + np.sin(t))

        msg = Float32()
        msg.data = float(force)

        self.pub.publish(msg)
        self.get_logger().info(f"Force published: {force:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = FSRSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

