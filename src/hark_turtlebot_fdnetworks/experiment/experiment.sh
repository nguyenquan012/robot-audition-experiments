#!/bin/sh
### NGUYEN Van Quan ###
### LARSEN - INRIA  ###



#################
### VARIABLES ###
#################


fast=_speed:=fast
slow=_speed:=slow
disc=_speed:=discontinuous
forward=_direction:=true
reverse=_direction:=false
start=_reposition:=start
end=_reposition:=end
turtlebot2=_turtlebot:=true
turtlebot1=_turtlebot:=false

#time to run one trial
t1=80s
#adjusting time
t2=20s

#Choose machine to run prepath_follow
machine1=$turtlebot1
#Choose machine to run HARK on
HARK=turtlebot-1
#choose machine to act as moving source 
movingsound=$turtlebot2

#experimental trials folder
expPath=`rospack find hark_turtlebot_fdnetworks`/experiment

#audio files in this computer
speech=`rospack find hark_turtlebot_fdnetworks`/audio/audio.wav
music=`rospack find hark_turtlebot_fdnetworks`/audio/korea_music.wav


#shell script file on turtlebot-1
script=/home/turtlebot/hark/localization_ROS.sh 


   


#################
### FUNCTIONS ###
#################

##
## save experimental data ## 
## HARK network, ROS msgs, ROS bag
##
save_trial() {   
	
	echo "save data of trial $1..."
	time_stamp=$1_$(date +%Y_%m_%d_%H_%M_%S)
	#mkdir -p "${expPath}/${time_stamp}"
	mkdir -p ~/hark_ros/exp/"${time_stamp}"/localization_trial_$1
	scp -r turtlebot@turtlebot-1:~/hark_ws/loc ~/hark_ros/exp/"${time_stamp}"/localization_trial_$1  #copy network output file from HARK
	#scp -r turtlebot@turtlebot-2:/hark_ws/loc "${expPath}/${time_stamp}"/localization_trial_$1  #copy network output file from HARK
	cp -r $(pwd)/trial_$1.bag ~/hark_ros/exp/"${time_stamp}"  #copy bag ros file
	echo "saved data of trial $1 in ~/hark_ros/exp/"${time_stamp}""
	echo "backup completed!"
}


##
## run experimental trials ##
##
## run_trial $1 $2 $3 $4 $5 $6 $7 
##
## $1 is trial's number
## $2 is speed mode
## $3 is direction
## $4 =1 recorded speech on, =0 off
## $5 =1 recorded music on, =0 off
## $6 =1 live water flow on, =0 off
## $7 =1 sources move, =0 sources not move

run_trial() {
	#echo "adjust turtlebot to right position"
	#rosrun turtlebot_move_simple turtlebot_move_simple $turtlebot1
	
	direction=$3
	#rem=$(( $1 % 2 ))
  	adjust_turtlebot
	run_HARK
	rosbag record -O trial_$1.bag tf /turtlebot/1/turtlespeed /turtlebot/1/slamomatic/pose /turtlebot/1/slamomatic/path &
	rosrun prepath_follow prepath_follow $2 $3 $machine1 &
	echo "rosrun prepath_follow prepath_follow $2 $3 $machine1"
	sleep 5s
			
	#play audio file 
	if [ $4 != "0" ]
	   then 
		echo "Active speech"
		aplay $speech  &  #turn speech on at turtlebot-2 position
        fi 

	if [ $5 != "0" ]
	   then 	
		echo "Active music"
		aplay $music  &  #turn music on
        fi 

	if [ $6 != "0" ]
	   then 
		echo "Active live flow water"
		echo "Please don't forget to turn the water on!!!"  &  #turn water on
        fi 

	if [ $7 != "0" ]  
	   then
	     echo "running turtlebot-2" & 
	     ssh turtlebot-2 '
		source ~/hark_ros/src/devel/setup.bash
		rosrun turtlebot_move_simple turtlebot_move_simple _turtlebot:=true _direction:=true &
		'&	
	fi 


	#run_HARK
           
	
	echo ""
	echo "running trial $1..."
	echo "trial $1 will last for $t1"
	sleep $t1 

	# stop prepath_follow node, music, ssh and save experimental data
	ps  #check running process
	killall -INT prepath_follow;killall -INT turtlebot_move_simple;killall ssh; killall -INT aplay 
	killall -INT rosbag 
	killall -INT record & sleep 0.5s; 
	save_trial $1
	echo "Finished trial $1!"
	ps  #check running process
	
	#rem=$(( $1 % 2 ))
	
	#adjust_turtlebot
}


##
## run HARK on turtlebot-2
##
run_HARK() {
	ssh -Y $HARK '
		cd hark_ws/loc		
		source ~/hark_ws/devel/setup.bash
		TF=`rospack find hark_turtlebot_fdnetworks`/networks/TF/kinect_tf.zip
		DEVICE=plughw:1,0
		echo ""
		echo "Location of your TF file : $TF"
		echo ""
		echo "ALSA Audio Device ID : $DEVICE"
		echo ""
		echo "rosrun hark_turtlebot_fdnetworks localization.n ${TF} ${DEVICE}"
		rosrun hark_turtlebot_fdnetworks localization.n ${TF} ${DEVICE} & sleep 80s;
		kill 0
		' &
            }

##
## adjust turtlebot-1 to fixed starting position of next trial
##
adjust_turtlebot() {
	#if [ $rem -ne 0 ]
	if [ $direction = $forward ]
	  then
	    echo "adjusting turtlebot-1 at start position"
	    rosrun turtlebot_move_simple turtlebot_move_simple $machine1 $forward & 
	else 
	   echo "adjusting turtlebot-1 at end position"
	   rosrun turtlebot_move_simple turtlebot_move_simple $machine1 $reverse & 
	fi
	sleep $t2 
	ps
	echo "Turtlebot-1 is ready for next trial"
}

##
## run all experiment
##

run_experiments() {

## run_trial $1 $2 $3 $4 $5 $6 $7 
##
## $1 is trial's number
## $2 is speed mode
## $3 is direction
## $4 =1 recorded speech on, =0 off
## $5 =1 recorded music on, =0 off
## $6 =1 live water flow on, =0 off
## $7 =1 sources move, =0 sources not move

# only recorded speech is on
run_trial 1 $slow $forward 1 0 0 0
run_trial 2 $slow $reverse 1 0 0 0
run_trial 3 $fast $forward 1 0 0 0
run_trial 4 $fast $reverse 1 0 0 0
run_trial 5 $disc $forward 1 0 0 0
run_trial 6 $disc $reverse 1 0 0 0

# only recorded music is on
run_trial 7 $slow $forward 0 1 0 0
run_trial 8 $slow $reverse 0 1 0 0
run_trial 9 $fast $forward 0 1 0 0
run_trial 10 $fast $reverse 0 1 0 0
run_trial 11 $disc $forward 0 1 0 0
run_trial 12 $disc $reverse 0 1 0 0

# recorded music + recorded speech + live water flow
run_trial 13 $slow $forward 1 1 1 0
run_trial 14 $slow $reverse 1 1 1 0
run_trial 15 $fast $forward 1 1 1 0
run_trial 16 $fast $reverse 1 1 1 0
run_trial 17 $disc $forward 1 1 1 0
run_trial 18 $disc $reverse 1 1 1 0

# only recorded speech + source moves
run_trial 19 $slow $forward 1 0 0 1  
run_trial 20 $slow $reverse 1 0 0 1
run_trial 21 $fast $forward 1 0 0 1
run_trial 22 $fast $reverse 1 0 0 1
run_trial 23 $disc $forward 1 0 0 1
run_trial 24 $disc $reverse 1 0 0 1

# recorded music + recorded speech + live water flow + sourve moves
run_trial 25 $slow $forward 1 1 1 1
run_trial 26 $slow $reverse 1 1 1 1
run_trial 27 $fast $forward 1 1 1 1
run_trial 28 $fast $reverse 1 1 1 1
run_trial 29 $disc $forward 1 1 1 1
run_trial 30 $disc $reverse 1 1 1 1
}


#################
###    MAIN   ###
#################


run_experiments   #1
sleep 2s 
echo "Finished all experiments!!!"
kill 0   #kill all processes except terminal

