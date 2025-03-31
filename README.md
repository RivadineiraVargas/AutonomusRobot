Proyecto de Navegación y Detección de Obstáculos en ROS 2

Descripción

Este proyecto implementa un sistema de navegación autónoma para un robot en un entorno simulado utilizando ROS 2 Humble. El robot emplea un sensor LiDAR para detectar paredes y obstáculos, y se controla mediante un nodo de movimiento inteligente.

Características

Control de movimiento: Un nodo move_robot.py gestiona la lógica de navegación del robot.

Detección de paredes y objetos: Se utilizan datos de un sensor LiDAR.

Publicación de mapas: Integración con rviz2 para visualizar el mapa y el LiDAR.

Simulación en Gazebo: Posibilidad de probar el sistema en un entorno simulado.


Instalación

1. Clona este repositorio en tu espacio de trabajo de ROS 2:

git clone https://github.com/tu-usuario/tu-repositorio.git
cd tu-repositorio


2. Instala las dependencias necesarias:

sudo apt update && rosdep install --from-paths src --ignore-src -r -y


3. Compila el paquete:

colcon build --symlink-install


4. Fuente el entorno:

source install/setup.bash



Uso

Iniciar la simulación en Gazebo

ros2 launch tu_paquete simulation.launch.py

Iniciar el nodo de movimiento del robot

ros2 run tu_paquete move_robot

Visualizar en RViz2

ros2 launch tu_paquete rviz.launch.py

Solución de problemas

Si el LiDAR devuelve solo valores 0.0, verifica que el frame_id sea correcto:

ros2 topic echo /scan | grep frame_id

Si RViz2 muestra "No map received", asegúrate de que el nodo de mapeo esté funcionando.


Contribución

Si deseas mejorar el proyecto, siéntete libre de hacer un fork y enviar un pull request.

Licencia

Este proyecto está bajo la licencia MIT.
