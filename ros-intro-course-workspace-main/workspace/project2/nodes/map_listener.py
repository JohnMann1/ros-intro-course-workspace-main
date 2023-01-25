#!/usr/bin/env python

# John Mann Project 2 Task 2 Part 2.1: frontier detection
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData

def grow(grid, i, j):
	# check edge cases
	if i < 10 or j < 10 or 384-i < 10 or 384-j < 10:
		return

	# non edge case
	for k in range(i-10, i):
		for l in range(j-10, j):
			grid.data[k*384 + l] = 100

class MapListener():
	subscriber = None
	publisher = None
	grid = None

	def __init__(self):
		self.init_node()
		self.init_subscriber()
		self.init_publisher()
		grid = OccupancyGrid()
		rospy.spin()


	def init_node(self):
		rospy.init_node('map_listener')

	def init_subscriber(self):
		print("subscribed")
		self.subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

	def init_publisher(self):
		print("publishing")
		self.publisher = rospy.Publisher('/frontiers_map', OccupancyGrid, queue_size=1)

	def map_callback(self, grid):
		# check to see if map is updated
		if grid == self.grid:
			# grids are equal so no update needed
			return

		# grids are not equal so update frontier
		self.grid = grid

		# grow obstacles using cspace in occupany grid
		#    loop through the grid and anywhere there is a 100
		#    change the next 10* left and down
		for i in range(0, 384):
			for j in range(0, 384):
				if grid.data[i*384 + j] == 100:
					grow(grid, i, j)

		# find frontiers:
		#    -1 --> unknown
		#     0 --> unoccupied
		#   100 --> occupied
		frontier = OccupancyGrid(grid.header, grid.info, [0]*(384*384))
		print(frontier.info.height)
		# walk the grid again
		# any cell that is unknown(-1) with a known, unoccupied(0) neighbor
		# is part of the frontier and gets value 100
		for i in range(1, 383):
			for j in  range(1, 383):
				if grid.data[i*384 + j] == -1:
					if grid.data[(i*384)+ j+1] == 0 or grid.data[(i*384)+j-1] == 0 or grid.data[(i*384)+j+1] == 0 or grid.data[(i*384)+j-1] == 0:
						frontier.data[i*384+j] = 100

		# visualize frontiers and publish as occupancy grid to
		# /frontiers_map, set frontier cells to 100 and non-f to 0
		while not rospy.is_shutdown():
			self.publisher.publish(frontier)

