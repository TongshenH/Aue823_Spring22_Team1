#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

"""This script allows the turtlebot to navigate while displaying an obstacle
avoidance algorithm.

Author(s): Grayson Byrd, Tongshen Huo, Parth Shinde, Saathana

Date: March 7, 2023
"""

# class Obstacle_Avoidance():
#     def __init__(self):
#         # Nodes, publishers, subscribers
#         rospy.init_node('wall_follower', anonymous=True)
#         self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
#         self.bot_control_publisher = rospy.Publisher("/cmd_vel", Twist)
#         rospy.sleep(2)
        
#         # Bot controls
#         # self.vel_msg = Twist()
#         # self.zero_controls()
#         # self.bot_control_publisher.publish(self.vel_msg)
        
#     def lidar_callback(self, msg):
#         # print(msg.ranges[0])
#         self.right = msg.ranges[-90:-20]
#         self.left = msg.ranges[20:90]
#         self.forward = msg.ranges[-20:]+msg.ranges[:20]
#         print(msg.ranges[90])
    
#     def run(self):
#         while not rospy.is_shutdown():
#             # print(self.left[0])
#             pass
            
# if __name__=='__main__':
#     OA = Obstacle_Avoidance()
#     OA.run()
        
        


PI = 3.1415

class Obstacle_Avoidance():
    def __init__(self):
        # Nodes, publishers, subscribers
        rospy.init_node('wall_follower', anonymous=True)
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.bot_control_publisher = rospy.Publisher("/cmd_vel", Twist)
        rospy.sleep(2)
        rospy.on_shutdown(self.shutdown)
        
        # Bot controls
        self.vel_msg = Twist()
        self.zero_controls()
        self.bot_control_publisher.publish(self.vel_msg)
        
    def shutdown(self):
        self.zero_controls()
        self.bot_control_publisher.publish(self.vel_msg)
        print('Shutting Down!')
        
    def lidar_callback(self, msg):
        # print(msg.ranges[0])
        self.right = msg.ranges[-90:-20]
        self.left = msg.ranges[20:90]
        self.forward = msg.ranges[-20:]+msg.ranges[:20]
        
    def make_decision(self):
        # Get sum of the left right and forward sectors
        l_sum, r_sum, f_sum = 0, 0, 0
        l_count, r_count, f_count = 0, 0, 0
        
        for d in self.left:
            if d > 15 or d < .05:
                continue
            l_sum += d
            l_count += 1
            
        for d in self.right or d < .05:
            if d > 15:
                continue
            r_sum += d
            r_count += 1
            
        for d in self.forward:
            if d < .15 and d > .02:
                # If there is an obstacle directly in front of the turtlebot, rotate cw
                print(d)
                return 1
            
        l_mean = l_sum/l_count if l_count != 0 else 1000
        r_mean = r_sum/r_count if r_count != 0 else 1000
        
        # 0 means to turn left, 2 to turn right, and 1 to rotate clockwise
        decision = 0 if l_mean < r_mean else 2
        
        return decision
    
    def rotate_cw(self):
        angular_vel = -0.2
        t1 = rospy.Time.now().to_sec()
        t2 = t1
        # Calculate time for 1/4 rotation, or PI/2
        duration = (PI/2)/abs(angular_vel)
        self.vel_msg.angular.z = angular_vel
        while (t2 - t1 < duration):
            self.bot_control_publisher.publish(self.vel_msg)
            t2 = rospy.Time.now().to_sec()
        self.zero_controls()
        
    def zero_controls(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        
        return
    
    def controls(self, decision):
        print(f'decision = {decision}')
        if decision == 0:
            self.zero_controls()
            self.vel_msg.linear.x = .2
            self.vel_msg.angular.z = .5
        elif decision == 2:
            self.zero_controls()
            self.vel_msg.linear.x = .2
            self.vel_msg.angular.z = -.5
        else:
            self.zero_controls()
            self.rotate_cw()
            self.vel_msg.linear.x = .2
            return
        
        self.bot_control_publisher.publish(self.vel_msg)
        # rospy.sleep(1)
        # self.zero_controls()
        # self.bot_control_publisher.publish(self.vel_msg)
        # rospy.sleep(1)
        return
    
    def run(self):
        while not rospy.is_shutdown():
            decision = self.make_decision()
            self.controls(decision)
    
if __name__=='__main__':
    OA = Obstacle_Avoidance()
    OA.run()