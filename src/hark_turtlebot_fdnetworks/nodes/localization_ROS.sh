#!/bin/sh
# arg $1 is name of localization output file from HARK

TF=`rospack find hark_turtlebot_fdnetworks`/networks/TF/kinect_tf.zip
DEVICE=plughw:2,0

#experiment results folder
EXP=`pwd`/experiment/test1
mkdir $EXP

#audio files on remote computer
AUDIO1=`rospack find hark_turtlebot_fdnetworks`/audio/audio1.wav
AUDIO2=`rospack find hark_turtlebot_fdnetworks`/audio/audio2.wav
AUDIO3=`rospack find hark_turtlebot_fdnetworks`/audio/audio3.wav

#shell script file on turtlebot-1
script=/home/turtlebot/hark/localization_ROS.sh 
#audito file on turtlebot-1
AUDIO4=/home/turtlebot/hark/audio/music1.wav

echo ""
echo "Location of your TF file : $TF"
echo ""
echo "ALSA Audio Device ID : $DEVICE"
echo ""

#run multiple comments on remoted computer
ssh turtlebot@turtlebot-1 bash -c "'
sh ${script} 
aplay ${AUDIO4}
'" & 

#xterm -title "App 1" -e "ssh turtlebot@turtlebot-1 aplay ${AUDIO4}" &
#xterm -title "App 1" -e "aplay ${AUDIO2}" & 
#xterm -title "App 2" -e "aplay ${AUDIO1}" &


#rosrun hark_turtlebot_fdnetworks localization.n ${TF} ${DEVICE}

#roslaunch prepath_follow prepath_follow 


#play multiple audio files simultaneously 
aplay ${AUDIO1} &
echo "" &
aplay ${AUDIO2} &
echo "" &
aplay ${AUDIO3} &

cp -i /usr/bin/hark-designer/userdata/networks/localization $EXP/$1

