#!/bin/sh

#start turtlebot-2
echo "start turtlebot-2" 
ssh turtlebot-2 '
	source ~/hark_ros/src/devel/setup.bash
	roslaunch prepath_follow start_turtlebot2.launch
	' &

