from setuptools import find_packages, setup

package_name = 'slam_obstacle_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/slam_obstacle_detection']),
        ('share/slam_obstacle_detection', ['package.xml']),
	('share/slam_obstacle_detection/launch', ['launch/slam_world.launch.py'])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='javier',
    maintainer_email='javier@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'vision_node = slam_obstacle_detection.vision_node:main',
	    'move_robot = slam_obstacle_detection.move_robot:main',
	    'wall_detector = slam_obstacle_detection.wall_detector:main',
        ],
    },
)
