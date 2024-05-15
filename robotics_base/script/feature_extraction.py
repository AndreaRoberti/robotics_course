#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class FeatureExtraction():
    def __init__(self):

        self.image_name_ = rospy.get_param('~image_name','/camera/color/image_rect_color')
        self.flip_image_name_ = rospy.get_param('~flip_image_name','/camera/color/image_rect_color/flip')
        self.sphere_topic_name_ = rospy.get_param('~sphere_topic_name','/sphere/pose')
        
        self.image_sub = rospy.Subscriber(self.image_name_, Image, self.rgb_img_callback)
        self.pose_sub = rospy.Subscriber(self.sphere_topic_name_, PoseStamped, self.pose_callback)
        self.image_pub_ = rospy.Publisher(self.flip_image_name_, Image, queue_size=1)


        self.bridge_ = CvBridge()

        self.sphere_pose_ = PoseStamped()

    def pose_callback(self,msg):
        self.sphere_pose_ = msg
        print(self.sphere_pose_)

    def rgb_img_callback(self, img):
        bgr_img = self.bridge_.imgmsg_to_cv2(img)  #convert ROS to OpenCV
        flipped_image = cv2.flip(bgr_img, 1) # IN SIM FLIP IMAGE

        hsv = cv2.cvtColor(flipped_image, cv2.COLOR_BGR2HSV)
        # Define the range of red color in HSV
        lower_red = np.array([0, 0, 0])
        upper_red = np.array([10, 255, 255])

        # Threshold the HSV image to get only red colors
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # Additional range for red in HSV (because red wraps around the 0/180 point in HSV)
        lower_red = np.array([160, 0, 0])
        upper_red = np.array([180, 255, 255])

        # Threshold the HSV image to get only red colors
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        # Combine the masks
        mask = mask1 + mask2
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(flipped_image, flipped_image, mask=mask)

        self.image_pub_.publish(self.bridge_.cv2_to_imgmsg(res, 'bgr8'))


#------------------------------------------------------------------

def main():
    rospy.init_node("feature_extraction")

    feature_extraction = FeatureExtraction()
    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
