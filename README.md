# PRM-A-
This is a gazebo simulation of a robot using A* with PRM. Use this code in catkin workspace and run obs_world.launch

ROS-Project

This is a project based on path planning and control theory of robots. What this means is that I need to plan a path from the given start to goal points through a given environment using one of the path planning algorithms. I then need to create a code for a controller for the robot. This will enable the robot to traverse the path point by point to reach the goal. The environment in which my project runs, consists of a series of static obstacles, and my robot. I used default turtlebot and the environment can be found in the world folder.

# Instructions to run
Create a package and place these folders appropriately in them.
1) Then run this command in termnial
       roslaunch PRM-A- obs_world.launch
       chmod +x planner.py
       chmod +x obstacle.py
2) To publish obstacles on the required topic, run:
       rosrun obstacle.py
3) To execute the path planner, run this in the terminal:
       rosrun planner.py
I have implemented a inbuilt PID controller inside the planner.py.
Voila! now you can see the turtelbot moving from 0,0 to the end goal position, you can feel free to nudge the values in the PID according to your needs.
