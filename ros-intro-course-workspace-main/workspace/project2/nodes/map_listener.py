#!/usr/bin/env python

# John Mann Project 2 Task 2
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sklearn.cluster import AgglomerativeClustering
from visualization_msgs.msg import MarkerArray, Marker

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
def coord_to_cell(cell, resolution):
	(x, y) = coord
	return [int(y / resolution), int(x / resolution)]

# grow
# input 2D array and 2 coordinates
# changes n cells to the left and down to be occupied
def grow(grid, x, y):
	n = 7
	# check edge case: left wall, bottom wall
	if x < n or y < n:
		return

	for i in range(x-n, x):
		for j in range(y-n, y):
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
	clustering = AgglomerativeClustering(n_clusters = k)
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
# returns the centroid for each cluters
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

	return centroids

# make_frontier_marker
def make_marker(loc, rgba, id, s = .1):
	f_marker = Marker()
	f_marker.header.frame_id = "map"
	f_marker.ns = "frontiers"
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
		  ColorRGBA(255,255,0,.5)]

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

class FrontierTracker():
	map_subscriber = None
	map_publisher = None
	marker_publisher = None
	grid = None
	negative_ones = None
	frontier = None
	resolution = None

	def __init__(self):
		self.init_node()
		self.init_frontier_publisher()
		self.init_marker_publisher()
		self.init_map_subscriber()
		self.grid = OccupancyGrid()
		self.negative_ones = 0
		self.frontier = OccupancyGrid()
		self.resolution = .05
		#while not rospy.is_shutdown():
		#	pass

	def init_node(self):
		rospy.init_node('map_listener')

	def init_map_subscriber(self):
		print("subscribed /map")
		self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
		rospy.spin()

	def init_frontier_publisher(self):
		print("publishing /frontiers_map")
		self.frontier_publisher = rospy.Publisher('/frontiers_map', OccupancyGrid, queue_size=1)

	def init_marker_publisher(self):
		print("publishing /frontier_markers")
		self.marker_publisher = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=1)

	def map_callback(self, grid):
		print("updating frontier")
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

		self.frontier = frontier

		# visualize frontiers and publish as occupancy grid to
		# /frontiers_map, set frontier cells to 100 and non-f to 0
		self.frontier_publisher.publish(self.frontier)

		# cluster frontiers
		n = 4                  # number of clusters
		frontier_points = find_frontier_points(frontier, frontier_count)
		cluster_list = find_clusters(frontier_points, n)

		# clear all markers in anticipation of new ones
		temp = MarkerArray()
		marker_array = [Marker(action=3)]
		temp.markers = marker_array
		self.marker_publisher.publish(temp)

		# make markers
		marker_array = make_marker_array(frontier_points, cluster_list, n)

		# find centroids
		centroids = calc_centroids(frontier_points, cluster_list, n)

		# make markers of these men
		centroid_markers = [None for i in range(n)]
		for i in range(n):
			id = i + len(marker_array.markers)
			centroid_markers[i] = make_marker(centroids[i],ColorRGBA(255, 255, 255, 1), id,.25)

		# add these men to the fray
		marker_array.markers = marker_array.markers + centroid_markers

		# publish markers
		# send them off to war
		self.marker_publisher.publish(marker_array)

