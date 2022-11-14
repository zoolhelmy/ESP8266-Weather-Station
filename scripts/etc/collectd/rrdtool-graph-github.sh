#!/bin/sh

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created		: 11 Nov 2022
#
# Function   	: RRDTool generate graph image and commit to GitHub
# 
# Run 			: Run as daily cron
#				: https://openwrt.org/docs/guide-user/base-system/cron
# 
# --------------------------------------------------------------------

# Variables
HOSTNAME="${COLLECTD_HOSTNAME:-$(cat /proc/sys/kernel/hostname)}" #
PATH_RRDFILE="/overlay/tmp/rrd/$HOSTNAME/exec-weather-mosquitto"  #
PATH_RRDTOOL="/usr/bin/rrdtool"                                   #
PATH_RRDIMG="/etc/collectd/images"                                #
GRAPH_WIDTH=840                                                   #
GRAPH_HEIGHT=100                                                  #
GRAPH_IMGTYPE="PNG"												  # RTFM options: PNG|SVG|EPS|PDF

# Plot graph function
plot_graph() {

	echo "Generate temperature $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/temperature-$1.png \
		-a PNG \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "$HOSTNAME: Temperature" \
		-v "Celcius (C)" \
		"DEF:1temperature_avg_raw=$PATH_RRDFILE/temperature.rrd:value:AVERAGE" \
		"CDEF:1temperature_avg=1temperature_avg_raw,0,+" \
		"DEF:3feelslike_avg_raw=$PATH_RRDFILE/feelslike.rrd:value:AVERAGE" \
		"CDEF:3feelslike_avg=3feelslike_avg_raw,0,+" \
		"LINE2:1temperature_avg#0158e8:Temperature (DHT22 Sensor) " \
		"GPRINT:1temperature_avg:MIN: Min\: %5.1lf C" \
		"GPRINT:1temperature_avg:AVERAGE: Avg\: %5.1lf C" \
		"GPRINT:1temperature_avg:MAX: Max\: %5.1lf C" \
		"GPRINT:1temperature_avg:LAST: Last\: %5.1lf C\l" \
		"LINE2:3feelslike_avg#219a7e:Feels Like (DHT22 Sensor) " \
		"GPRINT:3feelslike_avg:MIN: Min\: %5.1lf C " \
		"GPRINT:3feelslike_avg:AVERAGE: Avg\: %5.1lf C " \
		"GPRINT:3feelslike_avg:MAX: Max\: %5.1lf C " \
		"GPRINT:3feelslike_avg:LAST: Last\: %5.1lf C\l"
		
	echo "Generate humidy $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/humidity-$1.png \
		-a PNG \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "$HOSTNAME: Humidity" \
		-v "percentage (%)" \
		"DEF:1humidity_avg_raw=$PATH_RRDFILE/humidity.rrd:value:AVERAGE" \
		"CDEF:1humidity_avg=1humidity_avg_raw,0,+" \
		"LINE2:1humidity_avg#0158e8:Humidity" \
		"GPRINT:1humidity_avg:MIN: Min\: %5.1lf" \
		"GPRINT:1humidity_avg:AVERAGE: Avg\: %5.1lf" \
		"GPRINT:1humidity_avg:MAX: Max\: %5.1lf" \
		"GPRINT:1humidity_avg:LAST: Last\: %5.1lf\l"
		
	echo "Generate uv $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/uv-$1.png \
		-a PNG \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "$HOSTNAME: Ultraviolet Surface Energy" \
		-v "Energy (mW/cm²)" \
		"DEF:1uv_avg_raw=$PATH_RRDFILE/uv.rrd:value:AVERAGE" \
		"CDEF:1uv_avg=1uv_avg_raw,0,+" \
		"LINE2:1uv_avg#0158e8:Energy" \
		"GPRINT:1uv_avg:MIN: Min\: %5.1lf" \
		"GPRINT:1uv_avg:AVERAGE: Avg\: %5.1lf" \
		"GPRINT:1uv_avg:MAX: MAx\: %5.1lf" \
		"GPRINT:1uv_avg:LAST: Last\: %5.1lf\l"	

	echo "Generate duv $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/duv-$1.png \
		-a PNG \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "$HOSTNAME: Ultraviolet Index" \
		-v "Index" \
		"DEF:1duv_avg_raw=$PATH_RRDFILE/duv.rrd:value:AVERAGE" \
		"CDEF:1duv_avg=1duv_avg_raw,0,+" \
		"LINE2:1duv_avg#0158e8:Index" \
		"GPRINT:1duv_avg:MIN: Min\: %5.1lf" \
		"GPRINT:1duv_avg:AVERAGE: Avg\: %5.1lf" \
		"GPRINT:1duv_avg:MAX: Max\: %5.1lf" \
		"GPRINT:1duv_avg:LAST: Last\: %5.1lf\l"
}

# Github update function
update_github() {
	echo "Update_github WIP"
}

# Main
plot_graph day
plot_graph week
plot_graph month
plot_graph year
update_github

# EOF
