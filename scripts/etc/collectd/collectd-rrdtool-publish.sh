#!/bin/sh /etc/rc.common

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created	: 11 Nov 2022
#
# Function   	: collectd read the data and update RRDTool file.
# 
# Run		: Run as OS service or nohup
#		: https://openwrt.org/docs/techref/initscripts
# 
# --------------------------------------------------------------------

# Variables
START=10
STOP=15

HOSTNAME="${COLLECTD_HOSTNAME:-$(cat /proc/sys/kernel/hostname)}" #
INTERVAL=${COLLECTD_INTERVAL:-30}                                 #
INTERVAL=${INTERVAL%.*}                                           # OpenWRT collectd default 30 secs
RRDTOOL_IFID="exec-weather-mosquitto"				  #
PATH_COLLECTDDATA="/etc/collectd/data"				  # must be the same as mosquitto-subscriber.sh

# Main function
read_last_data() {

	# get the last value from Mosquitto & update the final data
	temperature=$(tail -n 1 $PATH_COLLECTDDATA/temperature.txt) 2>/dev/null
	temperature2=$(tail -n 1 $PATH_COLLECTDDATA/temperature2.txt) 2>/dev/null
	feelslike=$(tail -n 1 $PATH_COLLECTDDATA/feelslike.txt) 2>/dev/null
	humid=$(tail -n 1 $PATH_COLLECTDDATA/humid.txt) 2>/dev/null
	uv=$(tail -n 1 $PATH_COLLECTDDATA/uv.txt) 2>/dev/null
	duv=$(tail -n 1 $PATH_COLLECTDDATA/duv.txt) 2>/dev/null
	press=$(tail -n 1 $PATH_COLLECTDDATA/press.txt) 2>/dev/null
	presssea=$(tail -n 1 $PATH_COLLECTDDATA/presssea.txt) 2>/dev/null
	alt=$(tail -n 1 $PATH_COLLECTDDATA/alt.txt) 2>/dev/null
	altreal=$(tail -n 1 $PATH_COLLECTDDATA/altreal.txt) 2>/dev/null

	# collectd update values to RRDTool files
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/temperature\"  N:$temperature"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/temperature2\" N:$temperature2"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/feelslike\"    N:$feelslike"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/humidity\"     N:$humid"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/uv\"           N:$uv"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/duv\"          N:$duv"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/pressure\"     N:$press"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/pressuresea\"  N:$presssea"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/altitude\"     N:$alt"
	echo "PUTVAL \"$HOSTNAME/$RRDTOOL_IFID/altitudereal\" N:$altreal"
	
	# reset the data, if any of the services are down, the data will always zero
	rm -rf $PATH_COLLECTDDATA/*.txt
}

# Main infinite loop
main() {
	while true; do
		read_last_data
		sleep "$INTERVAL"
	done
}

echo start
main
 

# EOF

