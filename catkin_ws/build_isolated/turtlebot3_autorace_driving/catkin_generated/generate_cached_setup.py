# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/turtlebot3_autorace_msgs;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/turtlebot3_autorace_detect;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/turtlebot3_autorace_core;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/turtlebot3_autorace_camera;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/turtlebot3_autorace_2020;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/opencv_tests;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/apriltag_ros;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/image_geometry;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/assignment4_trackingandfollowing;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/cv_bridge;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/assignment3a_wallfollowingandobstacleavoidance;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/assignment2B_turtlebot3;/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/devel_isolated/turtlebot3_autorace_driving/env.sh')

output_filename = '/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/build_isolated/turtlebot3_autorace_driving/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
