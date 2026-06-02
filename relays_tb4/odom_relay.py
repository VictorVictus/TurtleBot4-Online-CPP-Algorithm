import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')

        # TB4 /odom is published as BEST_EFFORT. 
        # A 'Reliable' sub (default) will never see it.
        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # We publish to MATLAB as RELIABLE to keep the PC side stable.
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(Odometry, '/odom', self.callback, qos_in)
        self.pub = self.create_publisher(Odometry, '/pc_odom', qos_out)
        
        self.get_logger().info('Relay active: /odom (Best Effort) -> /pc_odom (Reliable)')

    def callback(self, msg):
        # Visual heartbeat in the terminal
        print(">", end="", flush=True) 
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
