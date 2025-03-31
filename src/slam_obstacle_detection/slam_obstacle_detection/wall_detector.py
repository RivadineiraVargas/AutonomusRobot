#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # Mensaje tipo Bool para indicar detección de pared

class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        self.wall_publisher = self.create_publisher(Bool, '/wall_detected', 10)
        self.get_logger().info("🟢 Nodo wall_detector iniciado")

    def lidar_callback(self, msg):
        """Detecta si hay una pared cerca y publica en /wall_detected"""
        front_ranges = [r for r in msg.ranges[160:200] if 0.1 < r < 3.5]  
        min_distance = min(front_ranges) if front_ranges else float('inf')

        WALL_THRESHOLD = 0.2

        wall_msg = Bool()
        wall_msg.data = min_distance < WALL_THRESHOLD  # True si hay una pared cerca
        self.wall_publisher.publish(wall_msg)

        if wall_msg.data:
            self.get_logger().warn(f"🟩 Pared detectada a {min_distance:.2f} metros")
            
        else:
            self.get_logger().info("✅ No hay paredes cercanas, se puede avanzar")

def main(args=None):
    rclpy.init(args=args)
    node = WallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


