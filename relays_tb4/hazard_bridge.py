#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from irobot_create_msgs.msg import HazardDetectionVector
from std_msgs.msg import String, Bool

class HazardBridge(Node):
    def __init__(self):
        super().__init__('hazard_bridge')

        # Create QoS profile that matches the robot's publisher
        # Using best effort reliability to match /hazard_detection
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
         
        # Subscribe with the matching QoS
        self.sub = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.hazard_callback,
            qos_profile)  # Use QoS profile instead of 10

        # Publisher with reliable QoS for MATLAB (String format)
        self.pub = self.create_publisher(String, '/matlab_hazard', 10)
        
        # NEW: Publisher for Bool bumper topic
        self.bump_pub = self.create_publisher(Bool, '/bumper_hit', 10)
        
        # Timer to clear bumper state after 0.5 seconds
        self.bump_timer = None
        self.bump_active = False

        self.get_logger().info('Hazard Bridge Node Started (with corrected QoS)')
        self.get_logger().info('Monitoring for BUMP hazards')
        self.get_logger().info('Publishing to /matlab_hazard (String) and /bumper_hit (Bool)')

    def hazard_callback(self, msg):
        hazards = []
        bump_detected = False

        for detection in msg.detections:
            if detection.type == 1:  # BUMP on your robot
                bump_detected = True
                hazard_type = 'BUMP'

                if hasattr(detection, 'header') and hasattr(detection.header, 'frame_id'):
                    location = detection.header.frame_id
                    if location.startswith('bump_'):
                        location = location[5:]  # Remove 'bump_'
                    hazards.append(f'{hazard_type}_{location}')
                    self.get_logger().info(f'BUMP DETECTED: {location}')
                else:
                    hazards.append(hazard_type)
                    self.get_logger().info('BUMP DETECTED: unknown location')

        # Publish String message to /matlab_hazard
        if hazards:
            msg_out = String()
            msg_out.data = ','.join(hazards)
            self.pub.publish(msg_out)
            self.get_logger().info(f'Published to /matlab_hazard: {msg_out.data}')

        # Publish Bool message to /bumper_hit
        if bump_detected and not self.bump_active:
            bool_msg = Bool()
            bool_msg.data = True
            self.bump_pub.publish(bool_msg)
            self.get_logger().info('Published to /bumper_hit: True')
            self.bump_active = True
            
            # Create timer to clear bumper after 0.5 seconds
            if self.bump_timer:
                self.bump_timer.cancel()
            self.bump_timer = self.create_timer(0.5, self.clear_bump)
    
    def clear_bump(self):
        bool_msg = Bool()
        bool_msg.data = False
        self.bump_pub.publish(bool_msg)
        self.get_logger().info('Published to /bumper_hit: False (cleared)')
        self.bump_active = False
        if self.bump_timer:
            self.bump_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = HazardBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
