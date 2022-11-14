
# ESP8266 Weather Station

NodeMCU ESP8266 with DHT22, BMP180, ML8511, MQ135 & KY037 sensors

ESP8266 have a limited single analog input pin A0, I have to sacrifice MQ135 & KY037 for the time being. The next version will have analog multiplexer to share pin A0.


## Features

- DHT22 - Temperature and humidity with digital interface
- BMP180 - Temperature, pressure and altitude with I2C interface
- ML8511 - UVA & UVB with analog interface
- MQ135 - Hazardous gas or VOC detector such as Ammonia (NH3), sulfur (S), Benzene (C6H6), CO2 etc. This is also come with analog interface.
- KY037 - Bangalore is noisy with their honking. So this microphone will plot the trend. Too bad this micrphone is not sensitive enough. Come with analog interface. Alternative part you can use is ICS43434.


## Architecture

MCU push sensor data through MQTT to both Thing Speak IOT cloud & my internal OpenWRT Mosquitto MQ. The next plan is to harvest MQ data with collectd and RRDTool for basic periodic graph.
## Assemble

Pin out reference to be updated.


## Setup

- Assemble the microprocessor.
- Install Arduino IDE. Refer to https://www.instructables.com/Steps-to-Setup-Arduino-IDE-for-NODEMCU-ESP8266-WiF/
- Plug into your USB port and ensure COM is ready.
- Update firmware sketch with the necessary wifi, IP address and ThingSpeak detail.
- Compile & push the firmware

## Daily graph

The graphs updated in daily basis from OpenWRT

![Daily temperature](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/master/images/temperature-day.png)

![Daily humidity](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/master/images/humidity-day.png)

![Daily ultraviolet surface energy](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/master/images/uv-day.png)

![Daily ultraviolet index](https://github.com/zoolhelmy/ESP8266-Weather-Station/blob/master/images/duv-day.png)

## Documentation

Worth to read through all the codes and issues that they have
- https://github.com/RobTillaart/ML8511
- https://github.com/RobTillaart/ML8511/issues/4 calibrate UV factor to get the right UV index. This will tally with your local weather channel.
- https://www.codrey.com/electronic-circuits/how-to-use-mq-135-gas-sensor/ sensor board come with the wrong resistor value. Need to replace the SMD with regular resistor.
- https://www.tindie.com/products/onehorse/ics43434-i2s-digital-microphone/
- https://github.com/knolleary/pubsubclient/ MQTT, the light MQ broker

## Authors

- [@zoolhelmy](https://www.github.com/zoolhelmy)

