#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
import torch

""" Author(s): Tongshen Hou, Parth Shinde, Saathana, Grayson Byrd

Description: This script is used to drive the turtlebot autonomously in the provided
gazebo world file.
"""

class Final(object):
    def __init__(self):
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f'working on {device}')
        
        # yolo model for object detection
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.yolo_model.cuda()
        self.yolo_model.eval()
        
        # subscribers and publishers
        rospy.init_node("final", anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        
        # OpenCV functionality
        self.bridge_object = CvBridge()
        
        # Bot controls
        self.starting_vel = .2
        self.vel_msg = Twist()
        self.zero_controls()
        self.vel_msg.linear.x = self.starting_vel
        self.vel_publisher.publish(self.vel_msg)
        
        # Obstacle Avoidance (OA) controller
        # Controller
        self.oa_Kp = .1
        self.oa_Ki = 0
        self.oa_Kd = 0
        self.oa_prev_error = 0
        self.oa_I = 0
        
        # Line Following Controller
        self.line_pid_count = 0
        self.line_I = 0
        self.line_prev_error = 0
        self.line_Kp = 0.03
        self.line_Ki = 0.0000
        self.line_Kd = 1
        
        # Class variables for line_following
        self.line_detected = False
        
        # For stop sign detection
        self.stopped = False
    
    # Subscriber callback functions
    def camera_callback(self, msg):
        try:
            self.cv2_image = cv2.resize(self.bridge_object.compressed_imgmsg_to_cv2(msg,desired_encoding="bgr8"), (400, 400), interpolation=cv2.INTER_AREA)
        except CvBridgeError as e:
            print(e)
            
    def lidar_callback(self, msg):
        self.left = self.normalize_lidar(list(msg.ranges[-60:-1]))
        self.right = self.normalize_lidar(list(msg.ranges[0:60]))
        
    def normalize_lidar(self, lidar_list):
        for idx, val in enumerate(lidar_list):
            normalized = self.sigmoid(val)
            normalized = 2* (normalized - .5)
            normalized = 1 - normalized
            lidar_list[idx] = normalized
            
        # the values now should be in between 0 and 1 with closer objects representing
        # a value closer to 1
        return lidar_list
    
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
        
    def zero_linear(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        
    def sigmoid(self, input):
        if input > 3.5:
            return 1
        return 1 / (1 + np.exp(-input))
    
    def avoid_obstacles(self):
        
        self.vel_msg.linear.x = self.starting_vel
        
        error = -(sum(self.right) - sum(self.left))
        
        # send error to PID controller
        angular_vel = self.oa_pid_control(error)
        
        # rotate
        self.rotate(angular_vel)
        
    def oa_pid_control(self, error):
        P = error
        self.oa_I = self.oa_I + error
        D = error - self.oa_prev_error
        self.oa_prev_error = error
        
        # when tuning, it was found that the integral and derivate were not necessary
        total = P*self.oa_Kp # + self.oa_I*self.oa_Ki + D*self.oa_Kd
        return total
    
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
        if np.sum(binary_img) > 0:
            self.line_detected = True 
        
        # calculate the centroid of the binary image
        M = cv2.moments(binary_img)
        
        # if no line is detected, return the function
        if M["m00"] == 0:
            self.zero_linear()
            self.vel_publisher.publish(self.vel_msg)
            return
        
        self.vel_msg.linear.x = self.starting_vel
        
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # add a circle in the camera images to verify correct location
        cv2.circle(self.cv2_image, (cX, cY + int(9*height//10)), 5, (0, 0, 255), -1)
        
        # calculate distance between centroid x value and center of the image
        error = width//2 - cX
        angular_vel = .05
        angular_vel *= self.line_pid_control(error)
        
        # turn towards the center of the line   
        self.rotate(angular_vel)
        
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
        
        return cropped_binary, height, width
    
    def rotate(self, angular_vel):
        self.zero_rotation()
        self.vel_msg.angular.z = angular_vel
        self.vel_publisher.publish(self.vel_msg)
        
    def line_pid_control(self, error):
        P = error
        self.line_I = self.line_I + error
        D = error - self.line_prev_error
        self.line_prev_error = error
        total = P*self.line_Kp + self.line_I*self.line_Ki + D*self.line_Kd
        return total
    
    def detect_stop_sign(self):
        
        # perform object detection on the camera image
        results = self.yolo_model(self.cv2_image)
        
        # get yolo results in pandas dataframe
        data = results.pandas().xyxy[0]

        # the stop sign is class 11. this determines if a stop sign was
        # detected in the camera frame
        is_stop = True if 11 in data['class'].values else False
        row = ...
        
        # if stop sign is detected, loop through to get the row of the class
        if is_stop:
            
            # find the row in the list of bounding boxes corresponding to the stop sign
            for i, row in enumerate(data.loc[:,'class']):
                if row == 11:
                    is_stop = True
                    row = data.iloc[i]
                    break
        
            # get the bounds of the box
            xmin = int(row.loc['xmin'])
            xmax = int(row.loc['xmax'])
            ymin = int(row.loc['ymin'])
            ymax = int(row.loc['ymax'])
            
            return xmin, xmax, ymin, ymax
        
        return -1, -1, -1, -1
        
    def draw_box(self, xmin, xmax, ymin, ymax):
        # top
        cv2.line(self.cv2_image, (xmin, ymax), (xmax, ymax), (0,255,0) ,2)
        # bottom
        cv2.line(self.cv2_image, (xmin, ymin), (xmax, ymin), (0,255,0), 2)
        # left
        cv2.line(self.cv2_image, (xmin, ymin), (xmin, ymax), (0,255,0), 2)
        # right
        cv2.line(self.cv2_image, (xmax, ymin), (xmax, ymax), (0,255,0), 2)
        
    def run(self):
        rospy.sleep(1)
        count = 0
        while not rospy.is_shutdown():
            # if no line is detected, run obstacle avoidance algorithm
            if self.line_detected == False:
                self.avoid_obstacles()
            
            # search for stop sign
            xmin, xmax, ymin, ymax = self.detect_stop_sign()
            
            # if stop sign found, draw bounding box
            if xmin != -1:
                self.draw_box(xmin, xmax, ymin, ymax)
                
                # use the size of the predicted bounding box as a threshold for when stopping
                # should take place. This prevents us from stopping if we see the stop
                # sign from far away.
                threshold = 90
                bounding_box_size = max(xmax-xmin, ymax-ymin)
                if bounding_box_size > threshold and self.stopped == False:
                    self.stopped = True
                    self.zero_controls()
                    rospy.sleep(3)
                    self.vel_msg.linear.x = self.starting_vel
                    self.vel_publisher.publish(self.vel_msg)
            
            # line following
            cropped_binary_img, height, width = self.convert_binary()
            self.follow_line(cropped_binary_img, height, width)
            cv2.imshow("Camera", self.cv2_image)
            cv2.waitKey(1)
            if count == 0:
                rospy.sleep(5)
                count += 1

def main():
    final = Final()
    final.run()
    
if __name__=='__main__':
    main()
    