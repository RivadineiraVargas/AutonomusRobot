import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javier/slam_obstacle_ws/install/slam_obstacle_detection'
