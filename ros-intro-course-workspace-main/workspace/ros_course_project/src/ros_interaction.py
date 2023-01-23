#!/usr/bin/env python3

from gui import GUI
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from threading import Thread

# ActionLib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class ROSInteraction:
    gui = None
    publisher = None
    client = None

    def __init__(self):
        self.init_node()
        self.init_publisher()
        self.init_subscriber()
        self.init_action_client()
        self.gui = GUI(
            direction_key_callback=self.direction_key_callback,
            coordinate_callback=self.coordinate_callback,
        )
        self.gui.run()

    def init_node(self):
        rospy.init_node(self)

    def init_publisher(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def init_subscriber(self):
        rospy.Subscriber('/odom', Odometry, self.direction_key_callback(Odometry))

    def init_action_client(self):
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()

    def direction_key_callback(self, direction):
        command = Twist()
        if direction == "forward":
            command.linear.x = 0.15
        elif direction == "backward":
            command.linear.x = -0.15
        elif direction == "left":
            command.linear.x = 0.1
            command.angular.z = 0.3
        elif direction == "right":
            command.linear.x = 0.1
            command.angular.z = -.03
        elif direction == "stop":
            command.linear.x = 0
            command.angular.z = 0

        self.publisher.publish(command)

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
 

    def subscription_callback(self, odometry):
        if self.gui is not None:
            pose = odometry.pose.pose
            self.gui.update_position(pose.position.x, pose.position.y, pose.position.z)
