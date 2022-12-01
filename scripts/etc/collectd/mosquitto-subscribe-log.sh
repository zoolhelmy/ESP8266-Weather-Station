#!/bin/sh

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created       : 23 Nov 2022
#
# Function      : mosquitto_sub to wait and collect logs from ESP8266 whenever it publish
#
# Run           : Run as OS service or nohup
#               : https://openwrt.org/docs/techref/initscripts
#               : https://unix.stackexchange.com/questions/463028/bash-script-to-log-mqtt-feed-to-txt-file
#
# --------------------------------------------------------------------

# Variables
MQ_BROKER="192.168.2.1"			             #
PATH_MQSUB="/usr/bin/mosquitto_sub"                  #
DATE=$(date '+%Y%m%d')                               #
PATH_COLLECTDLOG="/etc/collectd/log"                 #

read_mq_data() {

	$PATH_MQSUB -h $MQ_BROKER -t "weather/log" -F "%I %t %p" | tee -a $PATH_COLLECTDLOG/ESP8266_MQPrint_$DATE.log
	
}

main() {

	read_mq_data
	
}

main

# EOF