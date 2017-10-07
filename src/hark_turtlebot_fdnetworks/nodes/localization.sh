#!/bin/sh

TF=`rospack find hark_turtlebot_fdnetworks`/networks/TF/kinect_tf.zip
DEVICE=plughw:2,0

echo ""
echo "Location of your TF file : $TF"
echo ""
echo "ALSA Audio Device ID : $DEVICE"
echo ""

rosrun hark_turtlebot_fdnetworks localization.n ${TF} ${DEVICE} >HARK_log.txt
