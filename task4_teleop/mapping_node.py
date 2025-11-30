import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ForceMappingNode(Node):
    """
    Subscribes: 'pub_force' (raw force)
    Publishes: 'force_mapped' (0.0 to 1.0)
    Uses param: max_sensor_force
    """

    def __init__(self):
        super().__init__('force_mapping_node')

        self.declare_parameter('max_sensor_force', 20.0)
        self.max_force = self.get_parameter('max_sensor_force').get_parameter_value().double_value

        self.sub = self.create_subscription(
            Float32,
            'pub_force',
            self.force_callback,
            10
        )

        self.pub = self.create_publisher(
            Float32,
            'force_mapped',
            10
        )

        self.get_logger().info(
            f"ForceMappingNode started with max_sensor_force={self.max_force}"
        )

    def force_callback(self, msg: Float32):
        f = msg.data

        # map 0..max_force -> 0..1
        if self.max_force <= 0.0:
            mapped = 0.0
        else:
            mapped = f / self.max_force

        # clamp
        mapped = max(0.0, min(1.0, mapped))

        out = Float32()
        out.data = float(mapped)
        self.pub.publish(out)

        self.get_logger().info(f"Force={f:.2f} -> mapped={mapped:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ForceMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
