import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class LineFollower(object):
    def __init__(self):
        rospy.init_node("live_line_follower", anonymous=True)
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback)
        
    def camera_callback(self, msg):
        try:
            cv2_image = self.bridge_object.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        cv2.imshow("Camera Image", cv2_image)
        cv2.waitKey(1)
        
def main():
    line_follower_object = LineFollower()
    while not rospy.is_shutdown():
        pass

if __name__=='__main__':
    main()