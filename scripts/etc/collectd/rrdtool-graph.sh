#!/bin/sh

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created       : 11 Nov 2022
#
# Function      : RRDTool generate graph image and commit to GitHub
#
# Run           : Run as daily cron
#               : https://openwrt.org/docs/guide-user/base-system/cron
#
# --------------------------------------------------------------------

# Variables
PATH_RRDFILE="/overlay/tmp/rrd/ZoolGtw01/exec-weather-mosquitto"        #
PATH_RRDTOOL="/usr/bin/rrdtool"                                         #
PATH_RRDIMG="/etc/collectd/git/ESP8266-Weather-Station/images/graph"    #
PATH_WWW="/www/weather"							#
GRAPH_WIDTH=840                                                         #
GRAPH_HEIGHT=100                                                        #
GRAPH_IMGTYPE="PNG"                                                     # RTFM options: PNG|SVG|EPS|PDF

# Plot graph function
plot_graph() {

	echo "Generate temperature $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/temperature-$1.$(echo "$GRAPH_IMGTYPE" | awk '{print tolower($0)}') \
		-a $GRAPH_IMGTYPE \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "Weather: Temperature" \
		-v "Celcius (C)" \
		"DEF:1temperature_min_raw=$PATH_RRDFILE/temperature.rrd:value:MIN" \
		"CDEF:1temperature_min=1temperature_min_raw,0,+" \
		"DEF:1temperature_avg_raw=$PATH_RRDFILE/temperature.rrd:value:AVERAGE" \
		"CDEF:1temperature_avg=1temperature_avg_raw,0,+" \
		"DEF:1temperature_max_raw=$PATH_RRDFILE/temperature.rrd:value:MAX" \
		"CDEF:1temperature_max=1temperature_max_raw,0,+" \
		"DEF:3feelslike_min_raw=$PATH_RRDFILE/feelslike.rrd:value:MIN" \
		"CDEF:3feelslike_min=3feelslike_min_raw,0,+" \
		"DEF:3feelslike_avg_raw=$PATH_RRDFILE/feelslike.rrd:value:AVERAGE" \
		"CDEF:3feelslike_avg=3feelslike_avg_raw,0,+" \
		"DEF:3feelslike_max_raw=$PATH_RRDFILE/feelslike.rrd:value:MAX" \
		"CDEF:3feelslike_max=3feelslike_max_raw,0,+" \
		"LINE2:1temperature_avg#0158e8:Temperature (DHT22 Sensor) " \
		"GPRINT:1temperature_min:MIN: Min\: %5.1lf C" \
		"GPRINT:1temperature_avg:AVERAGE: Avg\: %5.1lf C" \
		"GPRINT:1temperature_max:MAX: Max\: %5.1lf C" \
		"GPRINT:1temperature_avg:LAST: Last\: %5.1lf C\l" \
		"LINE2:3feelslike_avg#9c2321:Feels Like (DHT22 Sensor) " \
		"GPRINT:3feelslike_min:MIN: Min\: %5.1lf C " \
		"GPRINT:3feelslike_avg:AVERAGE: Avg\: %5.1lf C " \
		"GPRINT:3feelslike_max:MAX: Max\: %5.1lf C " \
		"GPRINT:3feelslike_avg:LAST: Last\: %5.1lf C\l"

	echo "Generate humidy $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/humidity-$1.$(echo "$GRAPH_IMGTYPE" | awk '{print tolower($0)}') \
		-a $GRAPH_IMGTYPE \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "Weather: Humidity" \
		-v "percentage (%)" \
		"DEF:1humidity_min_raw=$PATH_RRDFILE/humidity.rrd:value:MIN" \
		"CDEF:1humidity_min=1humidity_min_raw,0,+" \
		"DEF:1humidity_avg_raw=$PATH_RRDFILE/humidity.rrd:value:AVERAGE" \
		"CDEF:1humidity_avg=1humidity_avg_raw,0,+" \
		"DEF:1humidity_max_raw=$PATH_RRDFILE/humidity.rrd:value:MAX" \
		"CDEF:1humidity_max=1humidity_max_raw,0,+" \
		"LINE2:1humidity_avg#0158e8:Humidity" \
		"GPRINT:1humidity_min:MIN: Min\: %5.1lf" \
		"GPRINT:1humidity_avg:AVERAGE: Avg\: %5.1lf" \
		"GPRINT:1humidity_max:MAX: Max\: %5.1lf" \
		"GPRINT:1humidity_avg:LAST: Last\: %5.1lf\l"

	echo "Generate uv $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/uv-$1.$(echo "$GRAPH_IMGTYPE" | awk '{print tolower($0)}') \
		-a $GRAPH_IMGTYPE \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "Weather: Ultraviolet Surface Energy" \
		-v "Energy (mW/cm²)" \
		"DEF:1uv_min_raw=$PATH_RRDFILE/uv.rrd:value:MIN" \
		"CDEF:1uv_min=1uv_min_raw,0,+" \
		"DEF:1uv_avg_raw=$PATH_RRDFILE/uv.rrd:value:AVERAGE" \
		"CDEF:1uv_avg=1uv_avg_raw,0,+" \
		"DEF:1uv_max_raw=$PATH_RRDFILE/uv.rrd:value:MAX" \
		"CDEF:1uv_max=1uv_max_raw,0,+" \
		"LINE2:1uv_avg#0158e8:Energy" \
		"GPRINT:1uv_min:MIN: Min\: %5.1lf" \
		"GPRINT:1uv_avg:AVERAGE: Avg\: %5.1lf" \
		"GPRINT:1uv_max:MAX: Max\: %5.1lf" \
		"GPRINT:1uv_avg:LAST: Last\: %5.1lf\l"

	echo "Generate duv $1ly graph..."
	$PATH_RRDTOOL graph $PATH_RRDIMG/duv-$1.$(echo "$GRAPH_IMGTYPE" | awk '{print tolower($0)}') \
		-a $GRAPH_IMGTYPE \
		-s NOW-1$1 \
		-e NOW-15 \
		-w $GRAPH_WIDTH \
		-h $GRAPH_HEIGHT \
		-t "Weather: Ultraviolet Index" \
		-v "Index" \
		"DEF:1duv_min_raw=$PATH_RRDFILE/duv.rrd:value:MIN" \
		"CDEF:1duv_min=1duv_min_raw,0,+" \
		"DEF:1duv_avg_raw=$PATH_RRDFILE/duv.rrd:value:AVERAGE" \
		"CDEF:1duv_avg=1duv_avg_raw,0,+" \
		"DEF:1duv_max_raw=$PATH_RRDFILE/duv.rrd:value:MAX" \
		"CDEF:1duv_max=1duv_max_raw,0,+" \
		"LINE2:1duv_avg#0158e8:Index" \
		"GPRINT:1duv_min:MIN: Min\: %5.1lf" \
		"GPRINT:1duv_avg:AVERAGE: Avg\: %5.1lf" \
		"GPRINT:1duv_max:MAX: Max\: %5.1lf" \
		"GPRINT:1duv_avg:LAST: Last\: %5.1lf\l"
		
}

# Main
main() {

	plot_graph hour
	plot_graph day
	plot_graph week
	plot_graph month
	plot_graph year
	cp $PATH_RRDIMG/temperature-day.png $PATH_WWW/
	cp $PATH_RRDIMG/humidity-day.png $PATH_WWW/
	
}

main

# EOF