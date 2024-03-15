#!/usr/bin/env python3
import numpy
import rospy

import tf

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class RoboticsBase():
    def __init__(self):

        self.pose_name_ = rospy.get_param('~pose_name','pose_name_default')
        
        self.pose_pub_ = rospy.Publisher(self.pose_name_, PoseStamped, queue_size=1)
        
        self.tf_listener_ = tf.TransformListener()
        self.br_ = tf.TransformBroadcaster()

    def get_transform(self):
        try:
            
            (trans,rot) = self.tf_listener_.lookupTransform('world', #target_Frame
                                       'base_link', #source frame
                                       rospy.Time(0)) #get the tf at first available time
            print(trans)
            print(rot)
            rot_matrix = tf.transformations.quaternion_matrix([rot[0],rot[1],rot[2],rot[3]])
            trasl_matrix = tf.transformations.translation_matrix( [trans[0], trans[1], trans[2]])
            T_matrix = numpy.dot(trasl_matrix, rot_matrix)
            print(T_matrix)
            print('-------')


            # http://docs.ros.org/en/lunar/api/tf/html/python/transformations.html
            self.br_.sendTransform((trans[0] + 1.0, trans[1] + 0.5, 1),
                     tf.transformations.quaternion_from_euler(0, 0, 1.57), # RPY
                     rospy.Time.now(),
                     'link_1',
                     "world")

                                    

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('No transform available')



    def update(self):
        self.get_transform()

        pose_ee = PoseStamped()
        pose_ee.header.frame_id = 'world'
        pose_ee.header.stamp = rospy.Time.now()
        pose_ee.pose.position.x = 0.0
        pose_ee.pose.position.y = 2.0
        pose_ee.pose.position.z = 1.0

        self.pose_pub_.publish(pose_ee)



#------------------------------------------------------------------

def main():
    rospy.init_node("robotics_base")

    robotics_base = RoboticsBase()
    
    rate = 100 # Hz
    ros_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        robotics_base.update()
        ros_rate.sleep()

if __name__ == '__main__':
    main()
