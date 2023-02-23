John Mann Project 3 Task 2

To run
- wall_follow.launch
	I had issues with getting the model load, so I used Sherif's solution. I don't think you will have any issues but if you do, do the follow
		1. mkdir ~/.gazebo/models
		2 cp -r ~/catkin_ws/src/project3/models/turtlebot3_plaza2/ ~/.gazebo/models/turtlebot3_plaza2
		

	then run 
		roslaunch project3 wall_follow.launch
		
To run the wall follow node, make sure the directory looks as follows
	~/catkin_ws/src/project3/src/main.py
	
	then your options are
		1. rosun project3 main.py test *table*
			- this will take in some file, such as q_table_manual.txt, and spawn the robot around the map running it with that q_table
			- my recommended test q_table which was learned is q_table_learning1000.txt
				-!!! rosrun project3 main.py test q_table_learning1000.txt
			
		2. rosrun project3 main.py learn *table* *folder*
			- this will have the robot learn from some given table and write into the given folder
			
		3. rosrun project3 main.py teleop
			- this just prints the state that the robot is in every 5 seconds
			- in another terminal run roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
			
During learning mode the graph will update every 10 episodes, but data for every episode is being collected.

I think something is broken with the learning, but it definitely cannot do the U turn