#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class FlipImage():
    def __init__(self):

        self.image_name_ = rospy.get_param('~image_name','/camera/color/image_rect_color')
        self.flip_image_name_ = rospy.get_param('~flip_image_name','/camera/color/image_rect_color/flip')
        
        self.image_sub = rospy.Subscriber(self.image_name_, Image, self.rgb_img_callback)
        self.image_pub_ = rospy.Publisher(self.flip_image_name_, Image, queue_size=1)


        self.bridge_ = CvBridge()

    def rgb_img_callback(self, img):
        bgr_img = self.bridge_.imgmsg_to_cv2(img)  #convert ROS to OpenCV
        flipped_image = cv2.flip(bgr_img, 1) # IN SIM FLIP IMAGE
        self.image_pub_.publish(self.bridge_.cv2_to_imgmsg(flipped_image, 'bgr8'))


#------------------------------------------------------------------

def main():
    rospy.init_node("flip_image")

    flip_image = FlipImage()
    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
