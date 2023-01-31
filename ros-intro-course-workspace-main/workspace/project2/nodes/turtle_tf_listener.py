#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

# John Mann Project 2 Task 1
class TurtleTransformListener():
	node = None
	listener = None

	def __init__(self):
		self.node = self.init_node()
		self.listener = self.init_listener()

		run()

	def init_node(self):
		self.node = rospy.init_node('turtle_tf_listener')

	def init_listener(self):
		self.listener = tf.TransformListener()

	def run():
		rate = rospy.Rate(10.0)

		while not rospy.is_shutdown():
			# look up /map to /base_footprint frame
			try:
				(trrans, rot) = listener.lookupTransform('/base_footprint', '/map', rospy.Time(0))
			except (tf.lookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			rate.sleep()

