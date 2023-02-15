#!/usr/bin/env python3

# John Mann Project 3 Task 1
# this robot follows the right wall at about .4m

# if roads were straight and had an infinitely long wall on the right,
# then I just built the first perfect self driving car

import rospy
import nav_msgs.msg
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Pose, Vector3, Point
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState #, SetModelState

#     \         /    /
# \    \       /   /
#   \   \     / /
#     \  \___/
# ______  | | ____________
#         |_|
#
#
#

####
#### Data Structure Stuff
####

linear_velocity = 0.1
angular_velocity = 0.2
num_actions = 3 # 0 turn left, 1 go straight, 2 turn right
starts = [[0,0,0,0,0,0], [1,1.5,0,0,.92,.37], [-2, -1.5,0,0,0,0], [1.8,0,0,0,.37,.92]]


# 0 is the closest, and different directions will have different granularity from there
# left: 0, 1 --> close, far TODO
# front: 0, 1 --> close, far TODO
# front_right: 0, 1, 2 --> close, medium, far
# right 0, 1, 2 --> close, medium, far
left_regions = 2
front_regions = 2
front_right_regions = 3
right_regions = 3

num_states = left_regions * front_regions * front_right_regions * right_regions

# object to hold states for q table
# this will hold 4 integers: left, front, front_right, right
# these integers correspond to the regions of distance in that direction
class State():
	def __init__(self, left, front, front_right, right):
		self.left = left
		self.front = front
		self.front_right = front_right
		self.right = right = right

# object to hold actions for q table
class Action():
	def __init__(twist):
		self.twist = twist


# q_table ## extremely helpful comment
class QTable():
	table = None
	actions = [Twist() for i in range(num_actions)]

	def __init__(self):
		self.init_actions()
		self.init_table()

	def init_actions(self):
		self.actions[0] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = angular_velocity)) # go left
		self.actions[1] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = 0)) # go straight
		self.actions[2] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = -1 * angular_velocity)) # go right

	# reads table in from file
	def init_table(self):
		f = open("./src/q_table.txt", "r")
		self.table = string_to_table(f.read())

	# q_table_lookup
	# input a state with region values
	# finds which vlaue in the table matches that state
	# returns the best option for action
	def table_lookup(self, state):
		n = find_in_state_table(state)

		# walk list to find the index with highest value
		max = -1
		max_index = 0
		for i in range(num_actions):
			if max < self.table[n][i]:
				max = self.table[n][i]
				max_index = i

		return self.actions[max_index]

	# update_q_table
	# TODO
	def update_table(self, reward):
		pass

	# write
	# saves the q_table in a file
	def write(self):
		file = open("./src/project3/src/q_table.txt", "w")
		file.write(table_to_string(self.table))
		file.close()

		# print nice
#		for i in range(num_states):
#			file.write(f"   {i}    {self.table[i][0]}    {self.table[i][1]}    {self.table[i][2]} \n")

####
#### couple of little helper functions to make code nicer
####

def reset():
	return rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

def set_model(pose, twist):
	state = ModelState(name = "turtlebot3_burger", pose = pose, twist = twist)
	return rospyService('/gazebo/set_model_state', state)

def pause():
	return rospy.ServiceProxy('/gazebo/pause_physics', Empty)

def unpause():
	return rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

def make_model_state(px = 0, py = 0, ox = 0, oy = 0, oz = 0, ow = 1):
	name = "turtlebot3_burger"
	pose = Pose()
	pose.position.x = px
	pose.position.y = py
	pose.orientation.x = ox
	pose.orientation.y = oy
	pose.orientation.z = oz
	pose.orientation.w = ow

	model_state = ModelState(model_name = name, pose = pose)
	return model_state

def set_spawn(i):
	return make_model_state(starts[i][0],starts[i][1],starts[i][2],starts[i][3],starts[i][4],starts[i][5])


# turns 2D array into string to write to file
# just makes it one big line, might change that later
def table_to_string(table):
	string = ""

	for i in range(num_states):
		for j in range(num_actions):
			string = string + str(table[i][j]) + " "
		string = string + '\n'

	return string

# turns string from file into q_table
# this one's kinda gross because numbers can be 1-3 digits
def string_to_table(string):
	table = [[0 for i in range(num_actions)] for i in range(num_states)]
	count = 0
	buffer = 0
	buffer_size = 0

	for i in range(len(string)):
		if string[i] == " ":
			# hit a space
			# add number to table
			table[count // num_actions][count % num_actions] = buffer # this some pretty cool math
			# reset for next number
			count += 1
			buffer = 0
			buffer_size = 0
		elif string[i] == "\n":
			pass
		else:
			# started or continued a number
			buffer_size += 1
			buffer = int(string[i-buffer_size+1:i+1])

	return table

# check_left
# determines closeness of 50 degrees to 90 degrees
def check_left(scan):
	left = min(scan[49:89])
	if left < .5:
		return 0 # close
	else:
		return 1 # far

# check_front
# determines closeness of 345 degrees to 15 degrees
def check_front(scan):
	front = min(scan[0:14] + scan[344:359])
	if front < .75:
		return 0 # close
	else:
		return 1 # far

# check_front_right
# determines closeness of 300 degrees to 340 degrees
def check_front_right(scan):
	front_right = min(scan[299:339])
	if front_right < .5:
		return 0 # close
	elif front_right >= .5 and front_right < .75:
		return 1 # medium
	else:
		return 2 # far

# check_right
# determines closeness of 260 degrees to 300 degrees
def check_right(scan):
	right = min(scan[259:299])
	if right < .4:
		return 0 # close
	elif right >= .4 and right < .6:
		return 1 # medium
	else:
		return 2 # far

# find_in_state_table
# input a state
# returns the table value
# basically just hard coding a bunch of states
# TODO use some kind of prime number system to make this better idk
def find_in_state_table(state):
	# creates a composite integer for values
	input = (state.left*(10**3)) + (state.front_right*(10**2)) + (state.front*(10)) + (state.right)

	# TODO this will need to grow to include left and front

	input = input % 100
	if input == 0: # right close, frontright close
		return 0
	if input == 1: # right medium, frontright close
		return 1
	if input == 2: # right far, frontright close
		return 2
	if input == 10: # right close, frontright medium
		return 3
	if input == 11: # right medium, frontright medium
		return 4
	if input == 12: # right far, frontright medium
		return 5
	if input == 20: # right close, frontright far
		return 6
	if input == 21: # right medium, frontright far
		return 7
	if input == 22: # right far, frontright far
		return 8

####
#### The Meat of It
####

# node to communication with gazebo
class WallFollower():
	# ros stuff
	laser_subscriber = None
	tf_listener = None
	move_publisher = None
	model_publisher = None

	# data stuff
	robot_position = None
	scan = None
	q_table = None
	restarts = None

	def __init__(self):
		self.init_node()
		self.init_laser_subscriber()
		self.init_tf_listener()
		self.init_q_table()
		self.init_move_publisher()
		self.init_model_publisher()
		self.restarts = 0

		while self.scan == None and not rospy.is_shutdown():
			rospy.sleep(1)

		###
		### Set the robot's starting position
		### there are 4 options in the global starts array
		###
		spawn = set_spawn(1)
		self.model_publisher.publish(spawn) # comment this line to stop spawning

		print("imma follow that wall now")
		# handles episodes for robot learning
		self.follow_wall()

		# save the q_table in case I'm an idiot and lose it somehow
		self.q_table.write()

	def init_node(self):
		rospy.init_node('wall_follower')

	def init_laser_subscriber(self):
		self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.store_scan)
		print("subscribed /scan", end=", ")

	def init_tf_listener(self):
		self.tf_listener = tf.TransformListener()
		self.tf_lookup() # initialize robot's position
		print("initialized tf_listener", end=", ")

	def init_q_table(self):
		self.q_table = QTable()
		print("initialized Q-table", end=", ")

	def init_move_publisher(self):
		self.move_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		print("publishing on /cmd_vel", end=", ")

	def init_model_publisher(self):
		self.model_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 1)
		print("publishing on /gazebo/set_model_state", end=", ")

	def init_model_states_subscriber():
		self.model_states_subscriber = rospy.Subscriber("/gazebo/model_states", )

	def store_scan(self, data):
		# just store the ranges for now
		# might want max/min range later idk
		self.scan = data.ranges

	def tf_lookup(self):
		# find robots position and orientation and store it in robot position
		# TODO this doesn't work because /map isn't a topic
		# doesn't cause any problems for task 1 though
		try:
			(trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			self.robot_position = (trans,rot)
			print(trans)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

	# get_state
	# uses the current scan to determine the state
	def get_state(self):
		# check each reason and assign an integer to send to state object
		left = check_left(self.scan)
		front = check_front(self.scan)
		front_right = check_front_right(self.scan)
		right = check_right(self.scan)

		# hand number to state object to get a value [0,35] for the table
		state = State(left, front, front_right, right)
		return state

	# follow_wall
	# handles the episodes for robot
	# checks if robot has crashed, if so resets to a new position
	def follow_wall(self):
		while not rospy.is_shutdown():
			# store position to check if move is successful
			before_position = self.robot_position

			# finds the current state
			state = self.get_state()

			# use state and take an action based on q_table
			self.take_step(state)
			rospy.sleep(.2)
			# TODO check to see if he's moved

	# take_step
	# inputs a state and publishes a movement message
	def take_step(self, state):
		pause()
		# check q_table
		action = self.q_table.table_lookup(state)

		# publish action
		self.move_publisher.publish(action)

		unpause()

		# TODO reward stuff
		result = None
		self.q_table.update_table(result)
