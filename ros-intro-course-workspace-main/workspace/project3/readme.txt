John Mann Project 3 Task 1

Follows a straigh wall well. Needs to come in at slightly right of perpendicular of the wall.

To run
First make sure you are in the direction "..."/project3. wall_follow.py needs to read in the q_table and the path is relative to /project3

my docker is having an issue where I have to remake the file for turtlebot3_plaza2 every time I start it up, so I follow the solution that Sherif gave. I don't think this will be an issue for anyone grading, but I'll include them in case.
	mkdir ~/.gazebo/models
	cp ~/catkin_ws/src/project3/models/turtlebot3_plaza2 ~/.gazebo/models/turtlebot3_plaza2

then I run the launch file
	roslaunch project3 wallfollow.launch

then to run the wall follower
	rosrun project3 main.py

The robot should spawn on the left at an angle and then start following the wall.

All of the wall follow protocol is in wall_follow.py. To change where the robot starts, change the value on line 292 to anything [0,4]. To stop the spawn when the file starts, comment out line 293. You can then teleop to any point and run the file to have it follow that wall on the right.
	roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

close --> 0
medium --> 1
far --> 2

for the front_right sensor:
	x < .4        --> 0
	.4 <= x < .6  --> 1
	x > .6	      --> 2

for the front_right sensor:
	x < .5        --> 0
	.5 <= x < .75 --> 1
	x > .75	      --> 2

State Table
    State | right | front_right |
      0   |   0   |      0      | 
      1   |   1   |      0      | 
      2   |   2   |      0      | 
      3   |   0   |      1      | 
      4   |   1   |      1      | 
      5   |   2   |      1      | 
      6   |   0   |      2      | 
      7   |   1   |      2      | 
      8   |   2   |      2      | 

Q_Table
    State | left | straight | right |
      0   |  100 |     0    |   0   |
      1   |  100 |     0    |   0   |
      2   |   0  |    100   |   0   |
      3   |   0  |    100   |   0   |
      4   |   0  |    100   |   0   |
      5   |   0  |     0    |  100  |
      6   |   0  |    100   |   0   |
      7   |   0  |     0    |  100  |
      8   |   0  |    100   |   0   |
