#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

# John Mann Project 2 Task 1
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # look up /map to /base_footprint frame
        try:
            (trans,rot) = listener.lookupTransform('/base_footprint', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rospy.loginfo("Location: " + str(trans))
        rate.sleep()