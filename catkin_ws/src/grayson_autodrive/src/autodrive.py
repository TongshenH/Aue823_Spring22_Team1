import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage

class AutoRace(object):
    def __init__(self):
        rospy.init_node("autorace", anonymous=True)
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        
    def camera_callback(self, msg):
        try:
            self.cv2_image = cv2.resize(self.bridge_object.compressed_imgmsg_to_cv2(msg,desired_encoding="bgr8"), (400, 200), interpolation=cv2.INTER_AREA)
        except CvBridgeError as e:
            print(e) 
        # self.get_binary_img()
        self.detect_lines()
    
    def get_binary_img(self):
        hsvMin = (0, 0, 216)
        hsvMax = (255, 255, 255)
        
        # convert to hsv
        hsv_image = cv2.cvtColor(self.cv2_image, cv2.COLOR_BGR2HSV)
        
        # apply thresholds
        mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
        res = cv2.bitwise_and(self.cv2_image, self.cv2_image, mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        ret, self.binary_img = cv2.threshold(gray, 70, 255, 0)
        
    def mask_img(self, img):
        # define the mask
        height = img.shape[0]
        width = img.shape[1]
        vertices = np.array([[0,height],[5*width//10,1*height//10], [width,height]])
        
        # mask image
        mask = np.zeros_like(img)
        cv2.fillPoly(mask,[vertices], color=(255, 255, 255))
        masked_image=cv2.bitwise_and(img,mask)
        return masked_image
    
    def crop_img(self, img):
        height, width = img.shape[0], img.shape[1]
        
        cropped_img = img[5*height//10:8*height//10]
        
        return cropped_img
        
        
    def detect_lines(self):
        # mask image
        # masked_img = self.mask_img(self.cv2_image)
        
        # crop image
        cropped_img = self.crop_img(self.cv2_image)
        
        # get grayscale image
        gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 150, apertureSize=3)
        cv2.imshow("Edges", edges)
        lines = cv2.HoughLinesP(edges,1,np.pi/180,10,minLineLength=5,maxLineGap=40)
        height = self.cv2_image.shape[0]
        for line in lines:
            x1,y1,x2,y2=line[0]
            cv2.line(self.cv2_image,(x1,y1+5*height//10),(x2,y2+5*height//10),(0,255,0),2)
            cv2.imshow("Lines", self.cv2_image)
            
    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            # self.get_binary_img()
            # self.detect_lines()
            # cv2.imshow("Binary", self.binary_img)
            cv2.waitKey(1)
        
if __name__=='__main__':
    autorace = AutoRace()
    autorace.run()