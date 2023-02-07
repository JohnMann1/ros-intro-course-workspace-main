#!/usr/bin/env python3

# John Mann Project 3 Task 1

import rospy
import nav_msgs.msg
import geometry_msgs.msg

# object to hold states for q table
class State():
	value = None


# object to hold actions for q table
class Action():


# node to communication with gazebo
class WallFollower():
	laser_subscriber = None

	def __init__(self):
		self.init_node()

	def init_node(self):
		rospy.init_node('wall_follower')

	def init_laser_subscriber(self):
		self.laser_subscriber = rospy.Subscriber("/scan", LaserScan)
		print("subscribed /scan")
