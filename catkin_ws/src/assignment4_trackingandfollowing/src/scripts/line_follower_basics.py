#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage

class LineFollower(object):
    def __init__(self):
        rospy.init_node("live_line_follower", anonymous=True)
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        
        # Bot controls
        self.vel_msg = Twist()
        self.zero_controls()
        self.vel_msg.linear.x = .75
        self.vel_publisher.publish(self.vel_msg)
        
        # For controller
        self.pid_count = 0
        self.I = 0
        self.prev_error = 0
        self.Kp = 0.4
        self.Ki = 0.0000
        self.Kd = 1
        
    def camera_callback(self, msg):
        try:
            self.cv2_image = cv2.resize(self.bridge_object.compressed_imgmsg_to_cv2(msg,desired_encoding="bgr8"), (400, 200), interpolation=cv2.INTER_AREA)
        except CvBridgeError as e:
            print(e) 
            
    def follow_line(self, binary_img, height, width):
        """ This function calculates the centroid of a binary image to determine
        the center of a path or line to follow. It then generates an error for a 
        controller by determining how far from the center of the camera image the
        centroid of the path/line is. It then calculates the angular velocity using
        a PID controller.

        Args:
            binary_img (numpy array): numpy array of the cropped binary image
            height (_type_): height of the non-cropped camera image
            width (_type_): width of the non-cropped camera image
        """
        # calculate the centroid of the binary image
        M = cv2.moments(binary_img)
        
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # add a circle in the camera images to verify correct location
        cv2.circle(self.cv2_image, (cX, cY + int(9*height//10)), 5, (255, 255, 255), -1)
        
        # calculate distance between centroid x value and center of the image
        error = width//2 - cX
        angular_vel = .05
        angular_vel *= self.pid_control(error)
        
        cv2.imshow("Cropped Binary", binary_img)
        
        # turn towards the center of the line   
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
        
    def pid_control(self, error):
        P = error
        self.I = self.I + error
        D = error - self.prev_error
        self.prev_error = error
        total = P*self.Kp + self.I*self.Ki + D*self.Kd
        # total = P*self.Kp
        return total
        
    def convert_binary(self):
        """Convert the initial camera image into a cropped binary image for line
        following.

        Returns:
            numpy array: cropped binary image
        """
        
        hsvMin = (20,120,120)
        hsvMax = (49,255,255)
        
        # convert to hsv
        hsv_image = cv2.cvtColor(self.cv2_image, cv2.COLOR_BGR2HSV)
        
        # apply thresholds
        mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
        res = cv2.bitwise_and(self.cv2_image, self.cv2_image, mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        
        ret, binary = cv2.threshold(gray, 70, 255, 0)
        
        # crop binary image
        height, width = binary.shape
        cropped_binary = binary[int(9*height//10):]
        
        cv2.imshow("Binary", binary)
        
        return cropped_binary, height, width
        
    def run(self):
        rospy.sleep(2)
        while not rospy.is_shutdown():
            cropped_binary_img, height, width = self.convert_binary()
            self.follow_line(cropped_binary_img, height, width)
            cv2.imshow("Camera Image", self.cv2_image)
            cv2.waitKey(1)
        
def main():
    line_follower_object = LineFollower()
    line_follower_object.run()

if __name__=='__main__':
    main()
    
"""
The commented out code below uses an inferior algorithm but was kept for comparison
purposes
"""

# class LineFollower(object):
#     def __init__(self):
#         rospy.init_node("line_follower", anonymous=True)
#         self.bridge_object = CvBridge()
#         self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_callback)
#         self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
#         rospy.sleep(2)
        
#         #Bot controls
#         self.vel_msg = Twist()
#         self.zero_controls()
#         self.vel_msg.linear.x = .2
#         self.vel_publisher.publish(self.vel_msg)
        
#         # for PID controller
#         self.pid_count = 0
#         self.I = 0
#         self.prev_error = 0
#         self.Kp = 0.1
#         self.Ki = 0.0000
#         self.Kd = 1
        
#     def camera_callback(self,data):
#         try:
#             # We select bgr8 because its the OpneCV encoding by default
#             self.cv_image = cv2.resize(self.bridge_object.compressed_imgmsg_to_cv2(data,desired_encoding="bgr8"), (400, 200), interpolation=cv2.INTER_AREA)
#         except CvBridgeError as e:
#             print(e)
#         cv2.imshow("Resized window", self.cv_image)
        
#     def follow_line(self, binary_img):
#         # calculate the amount of the path in the left and right halves of the 
#         # camera view
#         height = binary_img.shape[0]
#         width = binary_img.shape[1]
#         left = binary_img[:, :width//2].sum()
#         right = binary_img[:, width//2:].sum()
#         angular_vel = 1
#         if right < 1000 and left < 1000:
#             self.zero_rotation()
#             angular_vel = 0
#             # TODO: add locate path function
#         elif right == 0 and left != 0:
#             ratio = 10
#             angular_vel *= self.pid_conrol(ratio)
#         elif left == 0 and right != 0:
#             ratio = 10
#             angular_vel *= self.pid_conrol(ratio)
#             angular_vel *= -1
#         elif left > right:
#             ratio = left/right
#             angular_vel *= self.pid_conrol(ratio)
#         else:
#             ratio = -1*right/left
#             angular_vel *= self.pid_conrol(ratio)
            
#         self.rotate(angular_vel)
        
#     def rotate(self, angular_vel):
#         self.zero_rotation()
#         self.vel_msg.angular.z = angular_vel
#         self.vel_publisher.publish(self.vel_msg)
        
#     def zero_controls(self):
#         self.vel_msg.linear.x = 0
#         self.vel_msg.linear.y = 0
#         self.vel_msg.linear.z = 0
#         self.vel_msg.angular.x = 0
#         self.vel_msg.angular.y = 0
#         self.vel_msg.angular.z = 0
        
#     def zero_rotation(self):
#         self.vel_msg.angular.x = 0
#         self.vel_msg.angular.y = 0
#         self.vel_msg.angular.z = 0
        
#     def pid_conrol(self, error):
#         P = error
#         self.I = self.I + error
#         D = error - self.prev_error
#         self.prev_error = error
#         self.pid_count += 1
#         total = P*self.Kp + self.I*self.Ki + D*self.Kd
#         return total

#     def convert_binary(self):
#         # first introduce blur to smooth the edges of the road
#         smoothed_image = cv2.blur(self.cv_image, (50,50))
        
#         # min and max thresholds
#         hsvMin = (20,120,120)
#         hsvMax = (49,255,255)
        
#         # convert to hsv
#         hsv_image = cv2.cvtColor(smoothed_image, cv2.COLOR_BGR2HSV)
        
#         # apply thresholds
#         mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
#         res = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
#         gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        
#         ret, binary = cv2.threshold(gray, 70, 255, 0)
    
#         cv2.imshow("Binary Image", binary)
        
#         return binary
    
#     def convert_HSV(self):
#         # first introduce blur to smooth the edges of the road
#         smoothed_image = cv2.blur(self.cv_image, (10,10))
        
#         # min and max thresholds
#         hsvMin = (20,120,120)
#         hsvMax = (49,255,255)
        
#         # convert to hsv
#         hsv_image = cv2.cvtColor(smoothed_image, cv2.COLOR_BGR2HSV)
        
#         # apply thresholds
#         mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
#         res = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
        
#         return res
            
#     def run(self):
#         rospy.sleep(2)
#         while not rospy.is_shutdown():
#             binary_img = self.convert_binary()
#             self.follow_line(binary_img)
#             cv2.imshow("Binary Image", binary_img)
#             cv2.waitKey(1)

# def main():
#     line_follower_object = LineFollower()
#     line_follower_object.run()

# if __name__ == '__main__':
#     main()