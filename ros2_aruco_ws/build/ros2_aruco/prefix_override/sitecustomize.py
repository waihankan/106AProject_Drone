import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cc/ee106a/fa25/class/ee106a-ado/ros_workspaces/106AProject_Drone/ros2_aruco_ws/install/ros2_aruco'
