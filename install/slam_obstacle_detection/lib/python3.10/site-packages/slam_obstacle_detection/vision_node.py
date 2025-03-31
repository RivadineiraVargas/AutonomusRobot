#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import os

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/obstacle_info', 10)
        self.bridge = CvBridge()
        
        # Diccionario para almacenar objetos detectados y evitar duplicados
        self.detected_objects = {}  
        self.log_file = os.path.expanduser("~/contador_obstaculos.txt")

    def image_callback(self, msg):
        """Procesa la imagen, detecta obstáculos y los clasifica."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Encontrar contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)

            if w * h < 500:  # Filtrar objetos demasiado pequeños
                continue

            # Determinar tipo de objeto
            if len(approx) == 4:
                objeto = "Cuadrado"
            else:
                objeto = "Circular"

            # Evitar registrar el mismo objeto varias veces
            if not self.ya_existe_obstaculo(x, y):
                self.registrar_obstaculo(objeto, x, y)

                # Publicar en ROS
                msg = String()
                msg.data = f"{objeto} en ({x}, {y})"
                self.publisher.publish(msg)

            # Dibujar el objeto detectado en la imagen
            color = (0, 255, 0) if objeto == "Cuadrado" else (255, 0, 0)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(cv_image, objeto, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Mostrar imagen procesada
        #cv2.imshow("Detección de Objetos", cv_image)
        #cv2.waitKey(1)

    def registrar_obstaculo(self, nombre, x, y):
        """Registra un obstáculo en un archivo de texto y en el diccionario."""
        with open(self.log_file, "a") as f:
            f.write(f"{nombre} en ({x}, {y})\n")
        self.detected_objects[(x, y)] = nombre
        
        msg = String()
        msg.data = f"{nombre} en ({x}, {y})"
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Objeto detectado: {nombre} en ({x}, {y})")

    def ya_existe_obstaculo(self, x, y, min_distance=40):
        """Verifica si un obstáculo ya ha sido registrado en una posición similar."""
        for (px, py) in self.detected_objects.keys():
            distancia = math.sqrt((px - x) ** 2 + (py - y) ** 2)
            if distancia < min_distance:
                return True
        return False
    
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


