#!/usr/bin/env python

# John Mann Project 2 Task 3
# *Bill Hader Stefan voice* This file's got everything: frontier calculators, centroid publishers, and more hashtags than a 15 year old girl's instagram
# map_listener.py
# calcualtes frontiers from occupancy grid, segments frontiers into clusters
# finds centroids of clusters, picks a centroid to send as a goal for robot
import rospy
import actionlib
import tf
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sklearn.cluster import AgglomerativeClustering
from visualization_msgs.msg import MarkerArray, Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# count_negative_ones
# takes an OccupancyGrid
# returns the numhber of -1s
def count_negative_ones(grid):
	count = 0

	for i in range(len(grid.data)):
		if grid.data[i] == -1:
			count += 1

	return count


# grid_to_array
# takes an OccupancyGrid
# returns a 2D array
def grid_to_array(grid):
	arr = [[0] * grid.info.height] * grid.info.width

	for i in range(len(arr)):
		for j in range(len(arr[0])):
			arr[i][j] = grid.data[(grid.info.height*i) + j]

	return arr

# print1D
# debug helper
# prints the data set as a 2D array
def print1D(arr):
	for i in range(384):
		for j in range(384):
			print(arr[384*i+j],end=" ")
		print()

# cell_to_coord
# converts cell value to to meters
def cell_to_coord(cell, resolution):
	(x, y) = cell
	return [(y + .5) * resolution - 10, (x +.5) * resolution- 10] # might need to adjust for origin

# coord_to_cell
# converts meters to small
def coord_to_cell(coord, resolution):
	(x, y) = coord
	return [int(y + 10 / resolution), int(x + 10/ resolution)]

# grow
# input 2D array and 2 coordinates
# changes n cells to the left and down to be occupied
def grow(grid, x, y):
	n = 7
	# check edge case: left wall, bottom wall
	if x > 384 - n or y > 384 - n:
		return

	for i in range(x, x+n):
		for j in range(y, y+n):
			grid[i][j] = 100

# check_neighbors
# takes a 2D array, coordinates i and j, and a value to check
# looks at the 8 squares cardinally and diagonally around
# the cell given
# returns boolean
def check_neighbors(grid, i, j, value):
	if i == 0 or j == 0 or i > (len(grid[0])-1) or j > (len(grid)-1):
		return False
	# cardinal directions
	if grid[i+1][j]== value:
		return True
	if grid[i-1][j] == value:
		return True
	if grid[i][j+1] == value:
		return True
	if grid[i][j-1] == value:
		return True
	# diagonals
	if grid[i+1][j+1] == value:
		return True
	if grid[i+1][j-1] == value:
		return True
	if grid[i-1][j-1] == value:
		return True
	if grid[i-1][j+1] == value:
		return True

	return False

# find_clusters
# takes in an array of points and a number of expected clusters
# returns a list of clusters
def find_clusters(points, k):
	# give it to AgglomerativeCluster
	clustering = AgglomerativeClustering(n_clusters = k, distance_threshold = None)
	return clustering.fit(points).labels_


# segment_frontiers
# input an OccupancyGrid with values of 0 or 100, and a count for the total expected number
# returns the list of points
def find_frontier_points(frontier, total_count):
	height = frontier.info.height
	width = frontier.info.width

	# make an array of points
	frontier_points = [[0,0] for i in range(total_count)]
	current_count = 0

	for i in range(height):
		for j in range(width):
			if frontier.data[i*height + j] == 100:
				frontier_points[current_count] = [i,j]
				current_count += 1

	return frontier_points

# count_per_cluster
# takes an array of integers and returns the numbher of occurences
def count_per_cluster(cluster_list, n):
	occurences = [0 for i in range(n)]

	for i in range(len(cluster_list)):
		occurences[cluster_list[i]] += 1

	return occurences

# calc_centroid
# input list of clusters and corresponding list of points and number of clusters
# returns the centroid for each cluters and number of clusters
# !!! New behavior
# 	if there are less than 3 points in a cluster then send back
#	a new n so there will be less clusters next time
def calc_centroids(frontier_points, cluster_list, n):
	centroids = [[0,0] for i in range(n)] # list of clusters to return
	sums = [[0,0] for i in range(n)] # sums the points to be averaged
	lengths = [0 for i in range(n)] # counts occurences of each cluster

	for i in range(len(cluster_list)):
		lengths[cluster_list[i]] += 1
		sums[cluster_list[i]][0] += frontier_points[i][0] # x values
		sums[cluster_list[i]][1] += frontier_points[i][1] # y values

	for i in range(n):
		centroids[i][0] = sums[i][0] / lengths[i]
		centroids[i][1] = sums[i][1] / lengths[i]

	# walk list and set n to n-1 if any length < 6
#	k = 0
#	while k < len(lengths):
#		if lengths[k] < 6 and n != 1:
#			n = n - 1
#			k = len(lengths) + 1 # exit loop
#		k += 1

	return (centroids, n)

# make_frontier_marker
def make_marker(loc, rgba, id, s = .1, ns = "frontiers"):
	f_marker = Marker()
	f_marker.header.frame_id = "map"
	f_marker.ns = ns
	f_marker.type = 2
	f_marker.id = id
	f_marker.action = 0
	f_marker.color = rgba
	(x,y) = cell_to_coord((loc[0],loc[1]), .05)
	f_marker.pose.position.x = x
	f_marker.pose.position.y = y
	f_marker.pose.position.z = 0
	f_marker.pose.orientation.x = 0.0
	f_marker.pose.orientation.y = 0.0
	f_marker.pose.orientation.z = 0.0
	f_marker.pose.orientation.w = 1.0
	f_marker.scale.x = s
	f_marker.scale.y = s
	f_marker.scale.z = s
	#f_marker.lifetime = rospy.Duration(0)

	return f_marker


# generate_colors
# makes n distinct colors
def generate_colors(i, n):
	# this could be better, but I'm just going to hard code some values for now
	values = [ColorRGBA(255,0,0,.5),
		  ColorRGBA(0,255,0,.5),
		  ColorRGBA(0,0,255,.5),
		  ColorRGBA(255,255,0,.5),
		  ColorRGBA(100,100,100,.5),
		  ColorRGBA(100,100,100,.5),
		  ColorRGBA(100,100,100,.5),
		  ColorRGBA(100,100,100,.5),
		  ColorRGBA(100,100,100,.5),
		  ColorRGBA(100,100,100,.5),
		  ColorRGBA(100,100,100,.5)]

	return values[i]

# make_marker_array
# takes in frontier points,
def make_marker_array(frontier_points, cluster_list, n):
	marker_array = [None for i in range(len(cluster_list))]

	for i in range(len(frontier_points)):
		group = cluster_list[i]
		c = generate_colors(group, n)
		m = make_marker(frontier_points[i], c, i)
		marker_array[i] = m

	return MarkerArray(markers = marker_array)

##### new helper methods for task 3

# find_best_centroid
# input list of centroids and robot's location
# returns the best choice of centroid to go to
def find_best_centroid(centroids, position):
	# convert position to cells for ease
	cell_pos = coord_to_cell((centroids[0][0], centroids[0][1]), .05)

	least_distance = 100000000 # place holder
	least_index = 0 # index of the nearest point

	# walk list to find closest centroid
	for i in range(len(centroids)):
		d = distance(centroids[i][0], centroids[i][1], cell_pos[0], cell_pos[1])
		if d < least_distance:
			least_distance = d
			least_index = i


	return centroids[least_index]

# distance
# calculates the distance between 2 points
# advanced stuff :)
def distance(x1, y1, x2, y2):
	return ((x2-x1)**2 + (y2-y1)**2)**1/2

# fully_explored
# checks the number of unknown points and cuts off exploration at a certain value
# there's roughly  147,000 negative ones to start
# after exloring a 100 by 100 grid that'll put it around 147,000
# 138,350 seems to be pretty good for the error in some of the corners
def fully_explored(negative_ones):
	if negative_ones < 138350:
		return True
	return False

class FrontierExplorer():
	map_subscriber = None
	map_publisher = None
	marker_publisher = None
	action_client = None
	tf_listener = None
	grid = None
	negative_ones = None
	frontier = None
	resolution = None
	num_clusters = None
	centroids = None # this will be a list of Occupancy Grid Points
	robot_position: Pose() = None
	current_goal: Marker() = None

	def __init__(self):
		self.num_clusters = 4
		self.negative_ones = 0
		self.resolution = .05

		self.init_node()
		self.init_frontier_publisher()
		self.init_marker_publisher()
		self.init_map_subscriber()
		self.init_action_client()
		self.init_tf_listener()
		self.grid = OccupancyGrid()
		self.frontier = OccupancyGrid()
		self.robot_position = Pose()

		# let everything start up
		while self.centroids == None:
			rospy.sleep(1)

		# after initializing everything, begin exploring
		self.go_to_goal()

		# done exploring
		# clear all markers one last time
		self.marker_publisher.publish(MarkerArray(markers = [Marker(action=3)]))

		# gloat
		print("and Jesus wept for there were no worlds left to conquer -Dean Pelton")

	def init_node(self):
		rospy.init_node('map_listener')

	def init_map_subscriber(self):
		print("subscribed /map")
		self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.frontier_callback)

	def init_frontier_publisher(self):
		print("publishing /frontiers_map")
		self.frontier_publisher = rospy.Publisher('/frontiers_map', OccupancyGrid, queue_size=1)

	def init_marker_publisher(self):
		print("publishing /frontier_markers")
		self.marker_publisher = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=1)

	def init_action_client(self):
		print("Action Client listening to /move_base")
		self.action_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

		self.action_client.wait_for_server()

	def init_tf_listener(self):
		print("initializing tf_listener")
		self.tf_listener = tf.TransformListener()

		self.run_tf_listener()

	def coordinate_callback(self, x, y):
		pose = Pose()
		# need to readjust goal based on robot position and orientation
		# goal orientation will always be facing forward
		pose.position.x = x - self.robot_position.position.x
		pose.position.y = y - self.robot_position.position.y
		pose.orientation.x = 0 #self.robot_position.orientation.x
		pose.orientation.y = 0 #self.robot_position.orientation.y
		pose.orientation.z = 0 #self.robot_position.orientation.z
		pose.orientation.w = 1 #self.robot_position.orientation.w

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'base_footprint'
		goal.target_pose.pose = pose

		self.action_client.send_goal(goal)
		self.action_client.wait_for_result(rospy.Duration(12))

		#check location
		self.run_tf_listener()

		#check if he made it (or got close enough)
		if abs(x - self.robot_position.position.x) < .2 and abs(y - self.robot_position.position.y):
			return True
		else:
			return False

	def run_tf_listener(self):
		# check the robot's position every second and store it
		rate = rospy.Rate(1.0)

		# look up /map to /base_footprint
		try:
			(trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

			self.robot_position = Pose(position = Point(trans[0],trans[1],trans[2]), orientation = Quaternion(rot[0],rot[1],rot[2],rot[3]))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

		rate.sleep()

	def frontier_callback(self, grid):
		# check to see if map is updated
		if self.negative_ones == count_negative_ones(grid):
			return
		else:
			self.negative_ones = count_negative_ones(grid)

		# grids are not equal so update frontier
		self.grid = grid
		height = grid.info.height
		width = grid.info.width

		# grow obstacles using cspace in OccupancyGrid
		# create new 2D array and copy all values
		grown_grid = [[0 for i in range(height)] for j in range(width)]

		for i in range(height):
			for j in range(width):
				grown_grid[i][j] = grid.data[(i*height) + j]

		# loop through grid to grow values in 2D array
		for i in range(height):
			for j in range(width):
				if grid.data[i*height + j] == 100:
					grow(grown_grid, i, j)


		# find frontiers:
		#    -1 --> unknown
		#     0 --> unoccupied
		#   100 --> occupied
		frontier = OccupancyGrid(grid.header, grid.info, [0 for i in range(0,384*384)])
		frontier_count = 0

		# walk the grid again
		# any cell that is unknown(-1) with a known, unoccupied(0) neighbor
		# is part of the frontier and gets value 100
		for i in range(1, height-1):
			for j in range(1, width-1):
				if grown_grid[i][j] == -1:
					if check_neighbors(grown_grid, i, j, 0):
						frontier.data[i*height+j] = 100
						frontier_count += 1

		# if most of the map is explored, help trigger exit for go_to_goal loop
		if frontier_count < 5:
			self.negative_ones = 1 # very small

		self.frontier = frontier

		# visualize frontiers and publish as occupancy grid to
		# /frontiers_map, set frontier cells to 100 and non-f to 0
		self.frontier_publisher.publish(self.frontier)

		# cluster frontiers
		frontier_points = find_frontier_points(frontier, frontier_count)
		cluster_list = find_clusters(frontier_points, self.num_clusters)

		# clear all markers in anticipation of new ones
		# this will be the fronier and centroid markers, leaving the
		# 	current goal marker
		self.marker_publisher.publish(MarkerArray(markers=[Marker(action=3)])) # extremely constipated line

		# make markers
		marker_array = make_marker_array(frontier_points, cluster_list, self.num_clusters)

		# find centroids
		(c, n) = calc_centroids(frontier_points, cluster_list, self.num_clusters)
		self.centroids = c
		self.num_clusters = n

		# make markers of these men
		centroid_markers = [None for i in range(self.num_clusters)]
		for i in range(self.num_clusters):
			id = i + len(marker_array.markers)
			centroid_markers[i] = make_marker(self.centroids[i],ColorRGBA(255, 255, 255, 1), id,.25, "centroids")

		# add these men to the fray
		# include the current goal marker
		marker_array.markers = marker_array.markers + centroid_markers
		if self.current_goal != None:
			marker_array.markers = marker_array.markers + [self.current_goal]

		# publish markers
		# send them off to war
		self.marker_publisher.publish(marker_array)

	def go_to_goal(self):
		# explore map until no frontiers are left or ros stops
		while not rospy.is_shutdown() and not fully_explored(self.negative_ones):
			print("find new goal")

			# orient upwards
			pose = Pose()
			pose.position.x = 0
			pose.position.y = 0
			pose.orientation.x = 0
			pose.orientation.y = 0
			pose.orientation.z = 0
			pose.orientation.w = 1
			forward = MoveBaseGoal()
			forward.target_pose.header.frame_id = 'base_footprint'
			forward.target_pose.pose = pose
			self.action_client.send_goal(forward)
			self.action_client.wait_for_result(rospy.Duration(5))

			# get robot's position
			self.run_tf_listener()

			# find a goal to go to
			centroid = find_best_centroid(self.centroids, self.robot_position)

			# give robot new goal
			goal = (cell_to_coord((centroid[0], centroid[1]), self.resolution))

			# if goal hasn't changed since last time then go to origin
			#	(this usually happens when most of the map has been explored)
			if self.current_goal != None and self.current_goal.pose.position.x == goal[0] and self.current_goal.pose.position.y == goal[1]:
				print("I'm probably stuck :(, going to origin")
				self.action_client.cancel_all_goals()
				# change these so marker updates
				goal = [0,0]
				centroid = [0,0]

			print("goal: ", round(goal[0],2), round(goal[1],2))
			# add goal marker so it will be published with frontier points
			goal_marker = make_marker(centroid, ColorRGBA(0,0,0,.8), 1000, .3, "goal")
			self.current_goal = goal_marker

			# send it as a goal to move_base
			result = self.coordinate_callback(goal[0], goal[1])

			# check result
			if not result:
				# robot failed, try to orient upwards so that next goal will be sent correctly
				print("failure")

