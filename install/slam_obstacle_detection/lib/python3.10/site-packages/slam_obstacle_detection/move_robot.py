#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Mensaje para detección de pared
from sensor_msgs.msg import LaserScan  # Mensaje para datos del LiDAR
import random
import time
import numpy as np

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)        
        # Suscribirse al tópico de detección de paredes
        self.wall_subscription = self.create_subscription(
            Bool,
            '/wall_detected',
            self.wall_callback,
            10)
        
        # Suscribirse al LiDAR para detección de obstáculos
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        self.wall_detected = False
        self.wall_behind_detected = False
        self.obstacle_detected = False
        self.timer = self.create_timer(0.1, self.move_logic)
        
        self.last_map = None
        self.no_change_counter = 0
        self.max_no_change = 50
        
        
    def map_callback(self, msg):
        """ Verifica si el mapa ha cambiado. Si no cambia en varias iteraciones, el robot se detiene. """
        new_map = np.array(msg.data)
        if self.last_map is not None and np.array_equal(self.last_map, new_map):
            self.no_change_counter += 1
        else:
            self.no_change_counter = 0
        self.last_map = new_map        
    
    def wall_callback(self, msg):
        """Recibe información del nodo wall_detector"""
        self.wall_detected = msg.data
        if self.wall_detected:
            self.get_logger().info("🚨 Pared detectada, ajustando dirección...")

    def lidar_callback(self, msg):
        """Procesa los datos del LiDAR para detectar obstáculos"""
        ranges = msg.ranges
        front_distance = min(ranges[0:180])  # Rango frontal del LiDAR
        back_distance = min(ranges[180:360])
        
        obstacle_threshold = 0.7  # Distancia mínima para considerar un obstáculo
        
        self.obstacle_detected = front_distance < obstacle_threshold
        self.wall_behind_detected = back_distance < obstacle_threshold
        
        if self.obstacle_detected:
            self.get_logger().warn(f"⚠ Pared detectada adelante {front_distance:.2f}m, esquivando...")
            
        if self.wall_behind_detected:
            self.get_logger().warn(f"⚠ Pared detectada atras a {front_distance:.2f}m, esquivando...")
    
    def move_logic(self):
        """ Lógica de movimiento para evitar paredes, obstáculos y detenerse si el mapa no cambia """
        msg = Twist()

	    # Si el mapa no ha cambiado en 10 iteraciones, detener el robot
        if self.no_change_counter >= 10:
            self.get_logger().info("🛑 Mapa sin cambios, deteniendo el robot...")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return  # Sale de la función para evitar que siga enviando comandos de movimiento

        if self.wall_detected:
            self.get_logger().info("↪ Demasiado cerca de la pared, girando...")
            msg.linear.x = -0.2
            msg.angular.z = 0.5  # Gira para alejarse de la pared
            self.publisher_.publish(msg)
            time.sleep(1.0)  # Esperar a que gire

        elif self.wall_behind_detected:
            self.get_logger().info("🚨 Demasiado cerca de la pared detrás, girando para evitar...")
            msg.linear.x = 0.2  # Moverse hacia adelante
            msg.angular.z = 0.5  # Gira aleatoriamente
            self.publisher_.publish(msg)
            time.sleep(1.5)  # Espera para esquivar             

        elif self.obstacle_detected:
            self.get_logger().warn("↩ Obstáculo detectado, esquivando...")
            msg.linear.x = -0.2
            msg.angular.z = random.choice([0.4, -0.4])  # Gira aleatoriamente
            self.publisher_.publish(msg)
            time.sleep(1.5)  # Espera para esquivar

        else:
            self.get_logger().info("✅ No hay paredes ni obstáculos, avanzando recto")
            msg.linear.x = 0.2  # Velocidad normal
            msg.angular.z = 0.0
            self.publisher_.publish(msg)  

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

