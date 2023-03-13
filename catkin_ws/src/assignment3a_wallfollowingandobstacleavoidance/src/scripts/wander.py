#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def callback(msg):
    dataset = list(msg.ranges[0:359]) 

    front_sec = dataset[0:20] + dataset[-20:]

    front_sec = [x for x in front_sec if not x > 3]        
    front_sec = [x for x in front_sec if not x ==0]

    left_sec  = dataset[20:90]                                   
    left_sec = [x for x in left_sec if not x > 3]
    left_sec = [x for x in left_sec if not x ==0]

    right_sec= dataset[-90:-20]                                 
    right_sec = [x for x in right_sec if not x > 3]
    right_sec = [x for x in right_sec if not x ==0]

    #avg_front = sum(front_sec)/len(front_sec)
    front_min = min(front_sec)
    avg_right = sum(right_sec)/len(left_sec)
    avg_left = sum(left_sec)/len(front_sec)

    #print('Front',(avg_front))
    #print('Left',(avg_left))
    #print('Right',(avg_right))

    if front_min < 0.3:
        if avg_left>avg_right:                       
            velocity_msg.angular.z = 1
            velocity_msg.linear.x = 0.04
        elif avg_right>avg_left:                    
            velocity_msg.angular.z = -1
            velocity_msg.linear.x = 0.04  
        else:                                       
            velocity_msg.angular.z = 1
            velocity_msg.linear.x = -.01

    else:                                           
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = 0.1
    
    pub.publish(velocity_msg)


rospy.init_node('wander', anonymous=True)
velocity_msg=Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
scan = rospy.Subscriber('scan', LaserScan,callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(velocity_msg)
    rate.sleep()
    pass
