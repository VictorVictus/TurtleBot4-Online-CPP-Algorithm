import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import HazardDetectionVector
from std_msgs.msg import String


class HazardRelay(Node):

    def __init__(self):
        super().__init__('hazard_relay')

        self.sub = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.callback,
            10)

        self.pub = self.create_publisher(
            String,
            '/hazard_detection_safe',
            10)

        self.get_logger().info("Hazard relay started")

    def callback(self, msg):

        out = []

        try:
            for d in msg.detections:
                frame = d.header.frame_id
                out.append(frame)
        except Exception as e:
            self.get_logger().warn(str(e))

        # publish simple string list
        s = String()
        s.data = ",".join(out)

        self.pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = HazardRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
