import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_best_effort.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_best_effort)

        self.sub = self.create_subscription(
            Twist,
            '/pc_cmd_vel',
            self.callback,
            10
        )

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
