
# ESP8266 Weather Station

NodeMCU ESP8266 with DHT22, BMP180, ML8511, MQ135 & KY037 sensors

ESP8266 have a limited single analog input pin A0, I have to sacrifice MQ135 & KY037 for the time being. The next version will have analog multiplexer to share pin A0.

## Daily Updated Graph

The graphs updated in daily basis from OpenWRT RRDTool. More graphs for daily, weekly, monthly & yearly are available in [images](https://github.com/zoolhelmy/ESP8266-Weather-Station/tree/main/images/graph) folder.

![Daily temperature](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/graph/temperature-day.png?raw=true)

![Daily humidity](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/graph/humidity-day.png?raw=true)

![Daily ultraviolet surface energy](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/graph/uv-day.png?raw=true)

![Daily ultraviolet index](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/graph/duv-day.png?raw=true)

## Features

Sensors
- DHT22 - Temperature and humidity with digital interface
- BMP180 - Temperature, pressure and altitude with I2C interface
- ML8511 - UVA & UVB with analog interface
- MQ135 - Hazardous gas or VOC detector such as Ammonia (NH3), sulfur (S), Benzene (C6H6), CO2 etc. This is also come with analog interface.
- KY037 - Bangalore is noisy with their honking. So this microphone will plot the trend. Too bad this micrphone is not sensitive enough. Come with analog interface. Alternative part you can use is ICS43434.

Functional
- ESP8266 MQTT publish to IoT Cloud [ThingSpeak](https://thingspeak.com/channels/1927021).
- ESP8266 MQTT publish to OpenWRT (Custom wifi router), plot RRDTool [graph](https://github.com/zoolhelmy/ESP8266-Weather-Station/tree/main/images/graph).
- ESP8266 HTTP service request from internal network to get adhoc reading.

![adhoc reading](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/photo/Mobile_HTTP.jpg?raw=true)

## Architecture

- ESP8266 push sensor data through MQTT to both ThingSpeak IOT cloud & OpenWRT Mosquitto MQ. At any moment HTTP request is available for adhoc reading. 
- ThingSpeak is a real time graph and can be further analyze with MathLab.
- HTTP request is currently limited to internal network segment. Too bad, Airtel ISP disabled any port forwarding. Otherwise I can assign a subdomain eg weather.zoolhelmy.com with the help of Dynamic DNS for dynamic public IP and read it from anywhere instantly. 
- Meanwhile in OpenWRT, MQ data is further digested by collectd & RRDTool for basic periodic graph. The generated static graph as PNG is push to github on daily basis.


![adhoc reading](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/photo/Architecture.png?raw=true)

## Assemble

Pin out reference to be updated.

## Setup

- Assemble the microprocessor.
- Install Arduino IDE. Refer to https://www.instructables.com/Steps-to-Setup-Arduino-IDE-for-NODEMCU-ESP8266-WiF/
- Plug into your USB port and ensure COM is ready.
- Update firmware sketch with the necessary wifi, IP address and ThingSpeak detail.
- Compile & push the firmware.
- Setup OpenWRT statistics custom exec plugin. Definition file exec.js
- Setup ThingSpeak account.
- Setup mosquitto-subscribe script as OpenWRT service.
- Setup collectd-rrdtool-publish script as OpenWRT statistics exec plugin.
- Setup rrdtool-graph-github.sh script as daily cron.

## Photo

Full assembly 
![Full assembly](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/photo/ESP8266_full_assembly.jpg?raw=true)

Final rest place. Take note UV reading is not under the direct sun as we hardly go outside while in Bangalore. So its just to measure under the balcony
![Final rest place](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/main/images/photo/ESP8266_final_rest_place.jpg?raw=true)

More photo in [here](https://github.com/zoolhelmy/ESP8266-Weather-Station/tree/main/images/graph)

## Documentation

Worth to read through all the codes and issues that they have
- https://github.com/RobTillaart/ML8511
- https://github.com/RobTillaart/ML8511/issues/4 calibrate UV factor to get the right UV index. This will tally with your local weather channel.
- https://www.codrey.com/electronic-circuits/how-to-use-mq-135-gas-sensor/ sensor board come with the wrong resistor value. Need to replace the SMD with regular resistor.
- https://www.tindie.com/products/onehorse/ics43434-i2s-digital-microphone/
- https://github.com/knolleary/pubsubclient/ MQTT, the light MQ broker
- https://github.com/sqrwf/openwrt-collectd-exec-dslstats collectd, luci-statistic

## Authors

- [@zoolhelmy](https://www.linkedin.com/in/zoolhelmy/)
