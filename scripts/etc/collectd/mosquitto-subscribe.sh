#!/bin/sh

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created       : 11 Nov 2022
#
# Function      : mosquitto_sub to wait and collect logs from ESP8266 whenever it publish
#
# Run           : Run as OS service or nohup
#               : https://openwrt.org/docs/techref/initscripts
#               : https://unix.stackexchange.com/questions/463028/bash-script-to-log-mqtt-feed-to-txt-file
#
# --------------------------------------------------------------------

# Variables
MQ_BROKER="192.168.2.1"                              #
PATH_MQSUB="/usr/bin/mosquitto_sub"                  #
PATH_WWW="/www/weather"		       	             #
PATH_GITHUB="/etc/collectd/git"                      #
PATH_COLLECTDTMP="/etc/collectd/tmp"                 #
PATH_COLLECTDDATA="/etc/collectd/data"               # must be the same as collectd-exec-weather.sh

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

	#copy for html
	cp $PATH_COLLECTDTMP/*.txt $PATH_WWW/

	#copy for git
		cp $PATH_COLLECTDTMP/*.txt $PATH_GITHUB/ESP8266-Weather-Station/data/

	# move the data for collectd to pickup
	mv $PATH_COLLECTDTMP/*.txt $PATH_COLLECTDDATA/
	
}

# Main infinite loop
main() {

	while true; do
		read_mq_data
	done
	
}

main

# EOF