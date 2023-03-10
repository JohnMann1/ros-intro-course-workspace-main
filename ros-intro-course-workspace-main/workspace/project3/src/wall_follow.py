#!/usr/bin/env python3

# John Mann Project 3 Task 2
# this robot follows the right wall at about .4m

# if roads were straight and had an infinitely long wall on the right,
# then I just built the first perfect self driving car

import rospy
import nav_msgs.msg
import tf
import math
from matplotlib import pyplot
from random import random
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
angular_velocity = 0.5

starts = [[] for i in range(12)]
#starts[0] = [0,0,0,0,0,0] # origin
starts[0] = [1,1.5,0,0,.92,.37] # going down left wall
starts[1] = [1,1.5,0,0,.92,.37] # going down left wall
starts[2] = [-2, -2,0,0,0,0] # up from bottom right corner
starts[3] = [1.4,0,0,0,0,1] # going north near top
starts[4] = [2,.2,0,0,.67,.74] # top going left
starts[5] = [0, 2,0,0,.99,-.01] # top middle going down
#starts[6] = [.2,-1.3,0,0,-.62,.77] # top right going into corner
starts[6] = [-2,.5,0,0,0,1] # middle bottom going north
#starts[7] = [1.07,-1.31,0,0, .66, .74] # top right going towards i wall
starts[7] = [1,1.5,0,0,.92,.37] # going down left wall
starts[8] = [-1.8,-.5,0,0, -.84, .52] # bottom middle going east
starts[9] = [-1.93,1.72,0,0, -.71, .69] # bottom left going east
starts[10] = [-2,.5,0,0, 0, 1] # middle left going south
starts[11] = [0,1.5,0,0, 1, 0] # middle slightly less left going south


# 0 is the closest, and different directions will have different granularity from there
# left: 0, 1 --> close, far
# front: 0, 1 --> close, far
# front_right: 0, 1, 2 --> close, medium, far
# right 0, 1, 2, 3, 4 --> close, medium close, medium, medium far, far
left_regions = 3
front_regions = 3
front_right_regions = 2
right_regions = 3

num_states = left_regions * front_regions * front_right_regions * right_regions

# option of 5 actions
actions5 = [Twist() for i in range(5)]
actions5[3] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = angular_velocity)) # go left
actions5[4] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = .5 *angular_velocity)) # go left
actions5[0] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = 0)) # go straight
actions5[1] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = -.5 * angular_velocity)) # go right
actions5[2] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = -1 * angular_velocity)) # go right

# option of 3 actins
actions3 = [Twist() for i in range(3)]
actions3[0] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = angular_velocity)) # go left
actions3[1] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = 0)) # go straight
actions3[2] = Twist(linear = Vector3(x=linear_velocity), angular = Vector3(z = -1* angular_velocity)) # go right

actions = actions3
num_actions = len(actions)

# correct actions
# [l, f, fr, r, a]
test_cases = [[0 for i in range(5)] for i in range(7)]
test_cases[0] = [1,2,1,3,2]
test_cases[1] = [1,0,0,1,0]
test_cases[2] = [1,2,0,2,2]
test_cases[3] = [1,2,1,2,4]
test_cases[4] = [1,2,1,1,4]
test_cases[5] = [1,1,0,1,0]
test_cases[6] = [1,2,0,4,2]


# object to hold actions for q table
class Action():
	def __init__(twist):
		self.twist = twist


# q_table ## extremely helpful comment
class QTable():
	# table structure: table[l][f][fr][r][action]
	table = [[[[[100 for i in range(num_actions)] for i in range(right_regions)] for i in range(front_right_regions)] for i in range(front_regions)] for i in range(left_regions)]

	def __init__(self, file):
		self.init_table(file)

	# reads table in from file
	def init_table(self, file):
		if file != "q_table_blank.txt":
			path = "./src/q_tables/" + file
			f = open(path, "r")
			self.table = string_to_table(f.read())
		else:
			pass

	# q_table_lookup
	# input a state with region values
	# finds which vlaue in the table matches that state
	# returns the best option for action, and a boolean for if it was not random choice
	def table_lookup(self, state, e = 0):
		(l, f, fr, r) = state
		action_list = self.table[l][f][fr][r]
		a = None
		randy = random()
		# greed policy
		# if r is greater, pick the highest q table action
		# if r is smaller, pick a random one
		# epsilon gets smaller so starts picking the table more
		if randy > e:
			# find index of maximum value
			max_index = action_list.index(max(action_list))
#			print(state, ": ",calc_state(state), " --> ", action_list, ": ", max_index)
			return (max_index, True)
		else:
			return (math.floor(random()*len(actions)), False)

	# update_q_table
	# q_learning implementation
	def update_table(self, before, a, after, reward):
		(l1,f1,fr1,r1) = before
		(l2,f2,fr2,r2) = after

		learning = .2
		discount = .8

		old = self.table[l1][f1][fr1][r1][a]
		next_action = max(self.table[l2][f2][fr2][r2])
		new_q = old + learning * (reward + (discount * next_action) - old)

		self.table[l1][f1][fr1][r1][a] = round(new_q,2)
	# write
	# saves the q_table in a file
	def write(self, file):
		path = "./src/q_tables/" + file
		file = open(path, "w")
		file.write(table_to_string(self.table))
		file.close()


####
#### couple of little helper functions to make code nicer
####

def temp(list, i):
	for j in range(len(list)):
		list[j] = 0

	list[i] = 1

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

def calc_state(state):
	(l,f,fr,r) = state
	return l*left_regions+f*front_regions+fr*front_right_regions+r*right_regions

# writes the table as a CSV kinda
# never done a 5 deep loop before, hopefully never will again
def table_to_string(table):
	string = ""

	for i in range(left_regions):
		for j in range(front_regions):
			for k in range(front_right_regions):
				for l in range(right_regions):
					for m in range(num_actions):
						string = string + str(table[i][j][k][l][m]) + " "
					string = string +'\n'

	return string

# turns string from file into q_table
# this one's kinda gross because numbers can be 1-3 digits
def string_to_table(string):
	table = [[[[[0 for i in range(num_actions)] for i in range(right_regions)] for i in range(front_right_regions)] for i in range(front_regions)] for i in range(left_regions)]

	# clean list
	list = [0 for i in range(num_states*num_actions)]
	count = 0
	buffer = 0
	buffer_size = 0

	for i in range(len(string)):
		if string[i] == " ":
			# hit a space
			# add number to table
			list[count] = float(buffer)
			# reset for next number
			count += 1
			buffer = 0
			buffer_size = 0
		elif string[i] == "\n":
			pass
		else:
			# started or continued a number
			buffer_size += 1
			buffer = string[i-buffer_size+1:i+1]

	count = 0
	for i in range(left_regions):
		for j in range(front_regions):
			for k in range(front_right_regions):
				for l in range(right_regions):
					for m in range(num_actions):
						table[i][j][k][l][m] = list[count]
						count += 1


	return table

# check_left
# determines closeness of 30 degrees to 90 degrees
def check_left(scan):
	left = min(scan[29:89])
#	print("left: ", left)
	if left < .4:
		return 0 # close
	elif left >= .4 and left < .9:
		return 1 # medium
	else:
		return 2 # far

# check_front
# determines closeness of 330 degrees to 30 degrees
def check_front(scan):
	front = min(scan[0:29] + scan[329:359])
#	print("front:", front)
	if front < .3:
		return 0 # close
	elif front >= .3 and front < 1:
		return 1 # medium
	else:
		return 2 # far

# check_front_right
# determines closeness of 300 degrees to 330 degrees
def check_front_right(scan):
	front_right = min(scan[299:339])
#	print("front right:", front_right)
	if front_right < .6:
		return 0 # close
	else:
		return 1 # far

# check_right
# determines closeness of 250 degrees to 330 degrees
def check_right(scan):
	right = min(scan[249:329])
#	print("right:", right)

	if right < .2:
		return 0 # close
	elif right >= .2 and right < .6:
		return 1 # medium close
	else:
		return 2
#	if right < .2:
#		return 0 # close
#	elif right >= .2 and right < .3:
#		return 1 # medium close
#	elif right >= .3 and right < .4:
#		return 2 # medium
#	elif right >= .5 and right < .6:
#		return 3 # medium far
#	else:
#		return 4 # far

# check_move
def check_move(pose1, pose2):
	x = pose1[0]
	y = pose2[0]

	if abs(x[0] - y[0]) < .01 and abs(x[1] - y[1]) < .01:
		return False
	return True

def print_table(table):
	for i in range(left_regions):
		for j in range(front_regions):
			for k in range(front_right_regions):
				for l in range(right_regions):
					print((i,j,k,l), ": ", table[i][j][k][l])



# check_learning
# input state and action
# checks it against a few certain cases
# returns -1 if the state wasn't determinable, 0 if failed, 1 if correct
def check_learning(state, a):
	(l,f,fr,r) = state

	if l != 0 and f != 0 and fr == 0 and r == 1:
		if a == 1:
			return (1,1)
		else:
			return (1,0)
	if l != 0 and f == 0 and r == 2:
		if a == 0:
			return (0,1)
		else:
			return (0,0)
	if l == 0 and f == 0:
		if a == 2:
			return (2,1)
		else:
			return (2,0)
	if l != 0 and f != 0 and fr == 0 and r == 1:
		if a == 1:
			return (1,1)
		else:
			return (1,0)
	if fr == 1 and r < 2:
		if a == 2:
			return (2,1)
		else:
			return (2,0)
	if fr == 0 and r == 2:
		if a == 0:
			return (0,1)
		else:
			return (0,0)

	return (-1, -1)
# write_learning
# writes the learning array to a file just for safe keeping
def write_learning(graphs, folder):
	path = "/root/catkin_ws/src/project3/src/learning_graphs/"+folder+"/"
	# left
	f = open(path+"learning_left.txt", "w")
	f.write(str(graphs[0]))

	# straight
	f = open(path+"learning_straight.txt", "w")
	f.write(str(graphs[1]))

	# right
	f = open(path+"learning_right.txt", "w")
	f.write(str(graphs[2]))

	f.close()

# graph_learning
# graphs the overall learning_graph in real time
# should only get called every 10 or so episodes
def graph_learning(graphs):
	values = [0 for i in range(len(graphs[0]))]
	learning_graph = [[0,0] for i in range(len(graphs[0]))]

	# sum all of the learning
	for i in range(len(learning_graph)):
		correct = 0
		total = 0

		for j in range(num_actions):
			correct += graphs[j][i][0]
			total += graphs[j][i][1]
		learning_graph[i] = [correct,total]

	# get ratios
	for i in range(len(values)):
		if learning_graph[i][1] == 0:
			values[i] = 0
		else:
			values[i] = learning_graph[i][0]/learning_graph[i][1]

	pyplot.plot(values)
	pyplot.show(False)
	pyplot.pause(.001)

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
	learning_graphs = None

	def __init__(self, mode = "test", table=None, folder=""):
		self.init_node()
		self.init_laser_subscriber()
		self.init_tf_listener()
		self.init_move_publisher()
		self.init_model_publisher()
		self.restarts = 0
		self.learning_graphs = [[],[],[]] # holds left straight and right graphs

		while self.scan == None and not rospy.is_shutdown():
			rospy.sleep(1)

		if mode == "test":
			self.test_mode(table)
		elif mode == "teleop":
			self.teleop_mode()
		elif mode == "learn":
			self.learn_mode(table, folder)
		else:
			pass

	def init_node(self):
		rospy.init_node('wall_follower')

	def init_laser_subscriber(self):
		self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.store_scan)
		print("subscribed /scan", end=", ")

	def init_tf_listener(self):
		self.tf_listener = tf.TransformListener()
		self.tf_lookup() # initialize robot's position
		print("initialized tf_listener", end=", ")

	def init_q_table(self, file):
		self.q_table = QTable(file)
		print("initialized Q-table", end=", ")

	def init_move_publisher(self):
		self.move_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		print("publishing on /cmd_vel", end=", ")

	def init_model_publisher(self):
		self.model_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 1)
		print("publishing on /gazebo/set_model_state", end=", ")


	# teleop_mode
	# just prints the state its in
	def teleop_mode(self):
		print("Starting Teleop Mode")

		while not rospy.is_shutdown():
			print(self.get_state())
			rospy.sleep(10)

	# test_mode
	# runs episodes using the manual table
	# doesn't learn or change e
	# TODO for project 4 took out episode behavior in test mode
	def test_mode(self, table = "q_table_manual.txt"):
		print("Starting Test Mode")
		file = table
		# start from manual q_table
		self.init_q_table(file)

		# handles episodes for robot learning
#		episode = 0

		while not rospy.is_shutdown():
			### Set the robot's starting position
			### there are 12 options in the global starts array
#			spawn = set_spawn(episode % 12)
#			self.model_publisher.publish(spawn) # comment this line to stop spawning
#			self.start_episode(0, episode)
#			episode += 1
			state = self.get_state()
			self.take_step(state)

	def learn_mode(self, file, folder):
		print("Starting Learning Mode")

		if file == None or file == "None":
			file = "q_table_blank.txt"
		if folder != "":
			folder = folder+"/"

		# load q-table
		table = file
		print(table)
		self.init_q_table(table)

		episode = 0
		e = .9
		while not rospy.is_shutdown():
			# write a new q_table every 10 episodes
			if episode % 10 == 0:
				self.q_table.write(folder+"q_table_learning"+str(episode)+".txt")
				print_table(self.q_table.table)

			write_learning(self.learning_graphs, folder)
			graph_learning(self.learning_graphs)


			# pick a new start
			spawn = set_spawn(episode%12) # episode % len(starts))
			self.model_publisher.publish(spawn)
			self.start_episode(e, episode)

			# update e after episode until e == .1
			if e > .1:
				e = .99 * e

			episode += 1

			print("end of episode: ", episode)



	def store_scan(self, data):
		# just store the ranges for now
		# might want max/min range later idk
		self.scan = [data.range_max for i in range(len(data.ranges))]

		# TODO for project 4, clean list of zeroes
		for i in range(len(self.scan)):
			if data.ranges[i] > LaserScan.range_min:
				# fect the min finding
				self.scan[i] = data.ranges[i]

	def tf_lookup(self):
		# find robots position and orientation and store it in robot position
		try:
			(trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
			self.robot_position = (trans, rot)
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

		return (left, front, front_right, right)

	# start_episode
	# handles the episodes for robot
	# checks if robot has crashed, if so resets to a new position
	def start_episode(self, e, episode):
		stuck = 0
		stranded_count = 0
		update = 0
		for i in range(num_actions):
			self.learning_graphs[i] += [[0,0]]

		# constant to check if robot is far from everyting
		stranded = (left_regions-1, front_regions-1, front_right_regions-1, right_regions-1)

		start = rospy.get_rostime()
		current = rospy.get_rostime() # checking time to stop it from getting caught in a loop
		while not rospy.is_shutdown() and stuck < 50 and stranded_count < 50 and current.secs - start.secs < 60*(1.002**episode):
			# store position to check if move is successful
			self.tf_lookup()
			before_position = self.robot_position

			# finds the current state
			state = self.get_state()

			# use state and take an action based on q_table
			self.take_step(state, e, episode) #id%2==0) #TODO might want to update this less often
			rospy.sleep(.1)

			# check to see if he's moved
			self.tf_lookup()

			if not check_move(self.robot_position, before_position):
				# it hasn't moved
				stuck += 3
			else:
				# it moved
				stuck = 0

			if self.get_state() == stranded:
				stranded_count += .1
			else:
				stranded_count = 0
			update += 1
			current = rospy.get_rostime()

	# take_step
	# inputs a state, epsilon, and learning update boolean
	# TODO for project 4, removed all pause, sleep, and learning statements
	def take_step(self, state): #  , e, episode):
#		pause()
		# check q_table
		(a, not_randy) = self.q_table.table_lookup(state, e)

		# check learning
#		(correct_action, learn_value) = check_learning(state, a)
#		if learn_value != -1 and not_randy:
#			self.learning_graphs[correct_action][episode][0] += learn_value # counts total correct
#			self.learning_graphs[correct_action][episode][1] += 1 # counts total

		# publish action
		self.move_publisher.publish(actions[a])
#		unpause()

#		rospy.sleep(.1)

#		pause()
		# send off for q_learning
#		if e != 0 and episode%1 == 0:
#			reward = 20 # keeps values between 0 and 100
#			(l, f, fr, r) = self.get_state()
#			if r == 0 or r == right_regions-1 or f == 0 or l == 0:
#				reward = 0

			# give before and after states for q-learning
#			self.q_table.update_table(state, a, (l,f,fr,r), reward)

#		unpause()
