John Mann Project 2 Task 1

Project seems to work well. Start by running the three commands given in the assignment

roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch project2_task1 turtlebot3_navigation.launch

Gazebo and rviz will open as expected. The launch file will begin listing the current location of the turtle bot continuously.

In another terminal copy the commands found in /commands (command1.txt or command2.txt) and the robot will begin moving