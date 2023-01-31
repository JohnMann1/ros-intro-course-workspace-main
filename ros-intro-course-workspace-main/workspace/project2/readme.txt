John Mann Project 2 Task 2 ReadMe

Everything seems to work fine. My machine is a little slow updating the map but the video shows that it does that slowly as the robot is moving. These updates come whenevever new map data is sent.

All of the code I have written for this assignment is in the nodes/main.py and nodes/map_listener.py

main.py just calls map_listener.py. Map_listener.py calculates frontiers and publishes the points and centroids

to run the simulation I used the commands from task 1 to start
	roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
	roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
	roslaunch project2 turtlebot3_navigation.launch

before running main.py I had to install sklearn, but this was only one time whenever I opened a new docker image. You might not have this issue if it's installed already.
	pip3 install -U scikit-learn

then I run main.py which calls map_listener.py to calculate frontiers
	rosrun project main.py
