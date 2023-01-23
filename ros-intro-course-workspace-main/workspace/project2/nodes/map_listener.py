#!/usr/bin/env python

# John Mann Project 2 Task 2 Part 2.1: frontier detection
import rospy
from nav_msg.msg import OccupanyGrid

class MapListener():
    subscriber = None
    publisher = None
    
    def __init__(self):
        self.init_node()
        self.init_subscriber()

    def init_node(self):
        rospy.init_node('map_listener')

    def init_subscriber(self):
        rospy.Subscriber("/map", OccupanyGrid, map_callback)

    def map_callback(self, grid):
        # check to see if map is updated

        # grow obstacles using cspace in occupany grid

        # find frontiers:
        #    -1 --> unknown
        #     0 --> unoccupied
        #   100 --> occupied

        # visualize frontiers and publish as occupancy grid to
        # /frontiers_map, set frontier cells to 100 and non-f to 0
        pass

    
