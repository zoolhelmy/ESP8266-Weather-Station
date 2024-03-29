#!/bin/sh

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created	: 11 Nov 2022
#
# Function   	: mosquitto_sub to wait and collect data from ESP8266 whenever it publish and 
#	   	  copy the final value to a file. 
# 
# Run		: Run as OS service or nohup
#		: https://openwrt.org/docs/techref/initscripts
# 
# --------------------------------------------------------------------

# Variables
MQ_BROKER="192.168.2.1"                #
PATH_MQSUB="/usr/bin/mosquitto_sub"    #
PATH_COLLECTDTMP="/etc/collectd/tmp"   #
PATH_COLLECTDDATA="/etc/collectd/data" # must be the same as collectd-rrdtool-publish.sh

# Main function
# pickup the data whenever ESP8266 publish it
read_mq_data() {

	$($PATH_MQSUB -h $MQ_BROKER -t weather/temperature  -C 1 > $PATH_COLLECTDTMP/temperature.txt  ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/temperature2 -C 1 > $PATH_COLLECTDTMP/temperature2.txt ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/feelslike    -C 1 > $PATH_COLLECTDTMP/feelslike.txt    ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/humid        -C 1 > $PATH_COLLECTDTMP/humid.txt        ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/uv           -C 1 > $PATH_COLLECTDTMP/uv.txt           ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/duv          -C 1 > $PATH_COLLECTDTMP/duv.txt          ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/press        -C 1 > $PATH_COLLECTDTMP/press.txt        ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/presssea     -C 1 > $PATH_COLLECTDTMP/presssea.txt     ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/alt          -C 1 > $PATH_COLLECTDTMP/alt.txt          ) &
	$($PATH_MQSUB -h $MQ_BROKER -t weather/altreal      -C 1 > $PATH_COLLECTDTMP/altreal.txt      ) &

	# wait for each mosquitto_sub to read and terminate
	wait

	# move the data for collectd to pickup
	mv $PATH_COLLECTDTMP/*.txt $PATH_COLLECTDDATA/
}

# Main infinite loop
main() {
	while true; do
		read_mq_data
	done
}

echo start
main

# EOF

