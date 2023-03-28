#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower(object):
    def __init__(self):
        rospy.init_node("line_follower", anonymous=True)
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        rospy.sleep(2)
        
        #Bot controls
        self.vel_msg = Twist()
        self.vel_msg.linear.x = .2
        self.vel_publisher.publish(self.vel_msg)
        
    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # get binary image
        blur = cv2.bilateralFilter(self.cv_image, 9, 75, 75)
        mask = np.zeros(self.cv_image.shape, dtype=np.uint8)
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, 70, 255, 0)
        
        # perform morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # find distorted rectangle contour and draw onto a mask
        cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        rect = cv2.minAreaRect(cnts[0])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.cv_image, [box],0,(36,255,12),4)
        cv2.fillPoly(mask, [box], (255,255,255))
        
        cv2.imshow("Binary View", self.cv_image)
        cv2.waitKey(1)
            
    def run(self):
        rospy.sleep(2)
        while not rospy.is_shutdown():
            pass

def main():
    line_follower_object = LineFollower()
    line_follower_object.run()

if __name__ == '__main__':
    main()