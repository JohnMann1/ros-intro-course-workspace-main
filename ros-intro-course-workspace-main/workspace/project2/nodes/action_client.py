# John Mann Project 2 Task 1 
# simple action client with a lot of stuff copied from the tutorial
#! /usr/bin/env python

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def what():
    print("test")

class ActionClient:
    gui = None
    #publisher = None
    client = None

    def __init__(self):
        self.init_node()
        #self.init_publisher()
        #self.init_subscriber()
        self.init_action_client()
        # self.gui = GUI(
        #     direction_key_callback=self.direction_key_callback,
        #     coordinate_callback=self.coordinate_callback,
        # )
        # self.gui.run()
    
    def init_node(self):
        rospy.init_node("Action_Client")
    
    def init_action_client(self):
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()

    def coordinate_callback(self, x, y):
        thread = Thread(target=self.coordinate_callback_thread, args=(x, y))
        thread.start()

    def coordinate_callback_thread(self, x, y):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose

        self.client.send_goal(goal)

        self.client.wait_for_result()