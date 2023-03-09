#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

"""This script allows the turtlebot to employ emergency braking in the event that
it is about to crash into an obstacle.

Author(s): Grayson Byrd, Tongshen Huo, Parth Shinde, Saathana

Date: March 7, 2023
"""

class Emergency_Brake():
    def __init__(self, threshold):
        # Nodes, publishers, subscribers
        rospy.init_node('wall_follower', anonymous=True)
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.bot_control_publisher = rospy.Publisher("/cmd_vel", Twist)
        rospy.sleep(4)
        
        # Control parameters
        self.vel_msg = Twist()
        self.threshold = threshold
        self.emergency_brake_pulled = False
        
    def lidar_callback(self, msg):
        self.left = msg.ranges[-90:-20]
        self.right = msg.ranges[20:90]
        self.forward = msg.ranges[-20:]+msg.ranges[:20]
    
    def emergency_brake(self):
        self.zero_controls()
        self.bot_control_publisher.publish(self.vel_msg)
        self.emergency_brake_pulled = True
        
        return
    
    def check_for_obstacle(self, threshold):
        for d in self.forward:
            if d < threshold:
                self.emergency_brake()
                rospy.signal_shutdown('just cause')
            else:
                self.vel_msg.linear.x = .5
                self.bot_control_publisher.publish(self.vel_msg)
    
    def zero_controls(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        
        return
    
    def run(self):
        while not rospy.is_shutdown():
            if not self.emergency_brake_pulled:
                self.check_for_obstacle(self.threshold)
        
    
if __name__=='__main__':
    threshold = .4
    OA = Emergency_Brake(threshold)
    OA.run()