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
        self.zero_controls()
        self.vel_msg.linear.x = .2
        self.vel_publisher.publish(self.vel_msg)
        
    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        # processed_img = self.convert_HSV()
        # cv2.imshow("Processed Image", processed_img)
        # self.get_lines(processed_img)
        # cv2.imshow("Image window", self.cv_image)
        # cv2.waitKey(1)
        
    def follow_line(self, binary_img):
        # calculate the amount of the path in the left and right halves of the 
        # camera view
        height = binary_img.shape[0]
        width = binary_img.shape[1]
        left = binary_img[:, :width//2].sum()
        right = binary_img[:, width//2:].sum()
        angular_vel = .2
        if right == 0 and left == 0:
            pass
            # TODO: add locate path function
        elif right == 0 and left != 0:
            angular_vel *= 2.5
        elif left == 0 and right != 0:
            angular_vel *= -2.5
        elif left > right:
            ratio = left/right
            angular_vel *= ratio
        else:
            ratio = right/left
            angular_vel *= -ratio
            
        self.rotate(angular_vel)
        
    def rotate(self, angular_vel):
        self.zero_rotation()
        self.vel_msg.angular.z = angular_vel
        self.vel_publisher.publish(self.vel_msg)
        
    def zero_controls(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        
    def zero_rotation(self):
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def convert_binary(self):
        # first introduce blur to smooth the edges of the road
        smoothed_image = cv2.blur(self.cv_image, (50,50))
        
        # min and max thresholds
        hsvMin = (20,120,120)
        hsvMax = (49,255,255)
        
        # convert to hsv
        hsv_image = cv2.cvtColor(smoothed_image, cv2.COLOR_BGR2HSV)
        
        # apply thresholds
        mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
        res = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        
        ret, binary = cv2.threshold(gray, 70, 255, 0)
        
        cv2.imshow("Binary Image", binary)
        
        return binary
    
    def convert_HSV(self):
        # first introduce blur to smooth the edges of the road
        smoothed_image = cv2.blur(self.cv_image, (50,50))
        
        # min and max thresholds
        hsvMin = (20,120,120)
        hsvMax = (49,255,255)
        
        # convert to hsv
        hsv_image = cv2.cvtColor(smoothed_image, cv2.COLOR_BGR2HSV)
        
        # apply thresholds
        mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
        res = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
        
        return res
        
    def get_lines(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        
        # """ probabalistic hough transform """
        # lines = cv2.HoughLinesP(edges, 1, np.pi/180, 10, minLineLength=200, maxLineGap=30)
        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(self.cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        """ basic hough transform """
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100) 
        for line in lines:
            rho,theta=line[0]
            a=np.cos(theta)
            b=np.sin(theta)
            x0=a*rho
            y0=b*rho
            x1 = int(x0+1000*(-b))
            y1 = int(y0+1000*(a))
            x2 = int(x0-1000*(-b))
            y2 = int(y0-1000*(a))
            cv2.line(self.cv_image,(x1,y1),(x2,y2),(255,0,255),2)
            
    def run(self):
        rospy.sleep(2)
        while not rospy.is_shutdown():
            binary_img = self.convert_binary()
            self.follow_line(binary_img)
            cv2.imshow("Binary Image", binary_img)
            cv2.waitKey(1)

def main():
    line_follower_object = LineFollower()
    line_follower_object.run()

if __name__ == '__main__':
    main()