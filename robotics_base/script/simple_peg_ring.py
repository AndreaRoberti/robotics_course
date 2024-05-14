#!/usr/bin/env python3
import numpy
import rospy

import tf

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray


class SimplePegRing():
    def __init__(self):

        self.pose_name_ = rospy.get_param('~pose_name','pose_name_default')
        self.rings_topic_name_ = rospy.get_param('~ring_poses','rings/poses')
        self.pegs_topic_name_ = rospy.get_param('~pegs_poses','pegs/poses')
        
        self.pose_pub_ = rospy.Publisher(self.pose_name_, PoseStamped, queue_size=1)

        self.rings_sub_ = rospy.Subscriber(self.rings_topic_name_, PoseArray, self.rings_callback)
        self.pegs_sub_ = rospy.Subscriber(self.pegs_topic_name_, PoseArray, self.pegs_callback)
        
        self.tf_listener_ = tf.TransformListener()
        self.br_ = tf.TransformBroadcaster()

        self.rings_poses_ = PoseArray()
        self.pegs_poses_ = PoseArray()

    # [blue,cyan,red,yellow,purple,green]
    def rings_callback(self, msg):
        self.rings_poses_ = msg.poses
        # print(len(self.rings_poses_))

    # [left, right]
    def pegs_callback(self, msg):
        self.pegs_poses_ = msg.poses
        # print(len(self.pegs_poses_))


    def update(self):
        pose_ee = PoseStamped()
        pose_ee.header.frame_id = 'world'
        pose_ee.header.stamp = rospy.Time.now()
        pose_ee.pose.position.x = 0.0
        pose_ee.pose.position.y = 2.0
        pose_ee.pose.position.z = 1.0

        self.pose_pub_.publish(pose_ee)



#------------------------------------------------------------------

def main():
    rospy.init_node("simple_peg_ring")

    simple_peg_ring = SimplePegRing()
    
    rate = 100 # Hz
    ros_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        simple_peg_ring.update()
        ros_rate.sleep()

if __name__ == '__main__':
    main()
