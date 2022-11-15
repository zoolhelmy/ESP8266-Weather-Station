// -------------------------------------------------------------------------------
//
// Initial author: Zool Johan
// Created		 : 11 Nov 2022
//
// Function   	 : HTTP adhoc page, MQTT publish to ThingSpeak & OpenWRT Mosquitto
//				 : Wifi, HTTP, NTP, LED
//				 : DHT22, BMP180, ML8511, MQ135, KY037 sensors
// 
// -------------------------------------------------------------------------------

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ThingSpeak.h>
#include <ML8511.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <MQ135.h>

#define ANALOGPIN 				A0
#define DHTTYPE   				DHT22
#define seaLevelPressure_hPa 	1013.25

// built in LED
unsigned long currentMillis3 = 0;
unsigned long previousMillis3 = 0;

// Wifi
const char* ssid = "zoolhelmy.com";
const char* password = "xxxxx";
const char* MCUHostname = "WeatherStation";

// Static IP address
IPAddress local_IP(192, 168, 68, 99);
IPAddress gateway(192, 168, 68, 1);
IPAddress subnet(255, 255, 255, 0);

// http server
ESP8266WebServer server(80);

// NTP
const long utcOffsetInSeconds = 19800; // India Calcutta GMT +5.5
String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "time.google.com", utcOffsetInSeconds);

// MQTT Mosquitto
const char* mqttServer = "192.168.2.1";
const int mqttPort = 1883;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 30000; // Mosquitto publish interval
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT ThingSpeak
unsigned long currentMillis2 = 0;
unsigned long previousMillis2 = 0;
const long interval2 = 30000; // ThingSpeak publish interval
unsigned long ThingSpeakChannel = 123456;
const char* ThingSpeakAPIKey = "xxxxx";
WiFiClient TSClient;

// BMP180
Adafruit_BMP085 bmp;

// ML8511
ML8511 light(ANALOGPIN);

// DHT22
uint8_t DIGITALPIN = D8; // input pin
DHT dht(DIGITALPIN, DHTTYPE);

// KY-037
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

// MQ135
MQ135 gasSensor = MQ135(ANALOGPIN);

void setup() {
	
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println("ESP8366 Weather Station");

  setWiFi();
  setHTTP();
  setNTP();
  setMQTTMosquitto();
  setMQTTThingSpeak();
  setSensors();

  // setup built in LED
  pinMode(BUILTIN_LED, OUTPUT);
  
}

void loop() {
	
  // HTTP request handle
  server.handleClient();

  // MQTT Mosquitto publish every interval
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
	
    if (!client.connected()) {
      MQTTMosquittoReconnect();
    }

    Serial.println("MQTT Publish Mosquitto ----------");
    MQTTPublishMosquitto("weather/temperature", String(getTemperature()));
    MQTTPublishMosquitto("weather/temperature2", String(getTemperature2()));
    MQTTPublishMosquitto("weather/feelslike", String(getFeelsLike()));
    MQTTPublishMosquitto("weather/humid", String(getHumidity()));
    MQTTPublishMosquitto("weather/uv", String(getUV()));
    MQTTPublishMosquitto("weather/duv", String(getDUV()));
    MQTTPublishMosquitto("weather/press", String(getPressure()));
    MQTTPublishMosquitto("weather/presssea", String(getPressureSea()));
    MQTTPublishMosquitto("weather/alt", String(getAltitude()));
    MQTTPublishMosquitto("weather/altreal", String(getAltitudeReal()));
    // MQTTPublishMosquitto("weather/noise", String(getNoise()));
    // MQTTPublishMosquitto("weather/voc", String(getVOC()));
    // MQTTPublishMosquitto("weather/timestamp", getTimestamp());
  }  

  // MQTT ThingSpeak publish every interval
  currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
	
    Serial.println("MQTT Publish ThingSpeak ----------");
    MQTTPublishThingSpeak();
  }

  // Double wink the LED
  currentMillis3 = millis();
  if (currentMillis3 - previousMillis3 >= 5000) {
    previousMillis3 = currentMillis3;
    digitalWrite(BUILTIN_LED, LOW); 
    delay(50);
    digitalWrite(BUILTIN_LED, HIGH); 
    delay(50);
    digitalWrite(BUILTIN_LED, LOW); 
    delay(50);
    digitalWrite(BUILTIN_LED, HIGH); 
  }

}

// --------------------------------------------------------------------------------
// Setup section
// --------------------------------------------------------------------------------

void setWiFi () {
	
  // Wifi connect
  Serial.println("Connecting to ");
  Serial.println(ssid);

  if (!WiFi.config(local_IP, gateway, subnet, gateway)) {
    Serial.println("Wifi fixed IP failed");
  }
  WiFi.mode(WIFI_STA);
  WiFi.hostname(MCUHostname); // Not working with Lwip v2.x, it need v1.4
  WiFi.begin(ssid, password);

  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wifi is connected..!");
  Serial.print("IP Address: ");  Serial.println(WiFi.localIP());
  Serial.printf("Hostname: %s\n", WiFi.hostname());

  // WiFi auto reconnect
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
}

void setHTTP() {
	
  // HTTP server
  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
  server.begin();
  Serial.println("HTTP server started");
  
}

void setNTP() {
	
  // NTP
  timeClient.begin();
  
}

void setMQTTMosquitto() {
	
  // MQTT Mosquitto
  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
    Serial.print("Connecting to MQTT Mosquitto... ");
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");  
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }	
  
}

void setMQTTThingSpeak() {
	
  // MQTT Thing Speak
  ThingSpeak.begin(TSClient);
  
}

void setSensors() {
	
  // ML8511 UVA/UVB. Pin A0
  light.enable();
  light.setDUVfactor(0.005901288);   // default 1.61, from weather channel 11 (DUV) / (4.66 (UV) x 10,000) * 25
  light.setVoltsPerStep(3.3, 1024); // 10 bit DAC 1024 steps

  // DHT22 Temp, Humid. Pin D3
  pinMode(DIGITALPIN, INPUT);
  dht.begin();              

  // BMP180 Temp, Pressure, Alt. Pin SCL D1/SDA D2
  bmp.begin();

  // KY-037 Noise. Pin A0
  pinMode(ANALOGPIN, INPUT);

}

// --------------------------------------------------------------------------------
// MQTT Publish section
// --------------------------------------------------------------------------------

void MQTTPublishMosquitto(String PublishTopic, String PublishValue) { 
 
  char PublishTopicArr[PublishTopic.length()];
  PublishTopic.toCharArray(PublishTopicArr, PublishTopic.length() + 1);

  char PublishValueArr[PublishValue.length()];
  PublishValue.toCharArray(PublishValueArr, PublishValue.length() + 1);

  client.publish(PublishTopicArr, PublishValueArr);
  
}

void MQTTPublishThingSpeak() {

  ThingSpeak.setField(1, getTemperature2());
  ThingSpeak.setField(2, getHumidity());
  ThingSpeak.setField(3, getUV());
  ThingSpeak.setField(4, getDUV());
  ThingSpeak.setField(5, getPressureSea());
  ThingSpeak.setField(6, getAltitudeReal());
  // ThingSpeak.setField(1, getTemperature());
  // ThingSpeak.setField(5, getPressure());
  // ThingSpeak.setField(6, getAltitude());
  // ThingSpeak.setField(7, getNoise());
  // ThingSpeak.setField(8, getVOC());
  // ThingSpeak.setField(0, getTimestamp()); 
  
  
  int x = ThingSpeak.writeFields(ThingSpeakChannel, ThingSpeakAPIKey);
  if(x != 200){
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }  
  
}

void MQTTMosquittoReconnect() {
	
  while (!client.connected()) {
    Serial.print("MQTT Mosquitto re-connect... ");

    // Create a random client ID
    String clientId = "ESP8266-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.print(clientId);
      Serial.println(" re-connect is succesful");
    } else {
      Serial.print(clientId);
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
    }
  }
  
}

// --------------------------------------------------------------------------------
// Get Data section
// --------------------------------------------------------------------------------

float getTemperature() {

  float Temperature2 = bmp.readTemperature();
  Serial.print(Temperature2);
  Serial.println(" °C");
  
  return Temperature2;
  
}

float getTemperature2() {
	
  float Temperature = dht.readTemperature();
  Serial.print(Temperature);
  Serial.print(" °C, ");
  Serial.print(dht.readTemperature(true));
  Serial.print(" °F, feels like ");
  Serial.print(dht.computeHeatIndex(Temperature, dht.readHumidity(), false));
  Serial.println(" °C");
	
  return Temperature;
  
}

float getFeelsLike() {
	
  float FeelsLike = dht.computeHeatIndex(dht.readTemperature(), dht.readHumidity(), false);
  // Serial.print(dht.computeHeatIndex(Temperature, dht.readHumidity(), false));
  // Serial.println(" °C");
	
  return FeelsLike;
  
}

float getHumidity() {
	
  float Humidity = dht.readHumidity();
  Serial.print(Humidity);
  Serial.println("%");

  return Humidity;
  
}

float getUV() {
	
  float UV = light.getUV();
  // Serial.print(UV);
  // Serial.println(" mW/cm²");
  
  return UV;
  
}

float getDUV() {
	
  float UV = light.getUV();
  float DUV = light.estimateDUVindex(UV);  
  Serial.print(UV);
  Serial.print(" mW/cm², ");
  Serial.print(DUV);
  Serial.print(" index, ");
  Serial.print(light.isEnabled());
  Serial.print(" status, ");
  Serial.print(light.getDUVfactor());
  Serial.print(" DUV Factor, ");
  Serial.print(light.getVoltsPerStep());
  Serial.println(" V/step");

  return DUV;
  
}

float getPressure() {
  float Pressure = bmp.readPressure()/1000;
  Serial.print(Pressure);
  Serial.println(" kPa");

  return Pressure;
  
}

float getPressureSea() {
  float PressureSea = bmp.readSealevelPressure()/1000;
  Serial.print(PressureSea);
  Serial.println(" kPa");

  return PressureSea;
  
}

float getAltitude() {
	
  float Altitude = bmp.readAltitude();
  Serial.print(Altitude);
  Serial.println(" meters");

  return Altitude;
  
}

float getAltitudeReal() {
	
  float AltitudeReal = bmp.readAltitude(seaLevelPressure_hPa * 100);
  Serial.print(AltitudeReal);
  Serial.println(" meters");

  return AltitudeReal;
  
}

float getNoise() {
	
  // Noise in dB
  unsigned long startMillis = millis();                   // Start of sample window
  float peakToPeak = 0;                                  // peak-to-peak level

  unsigned int signalMax = 0;                            //minimum value
  unsigned int signalMin = 1024;                         //maximum value

  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(ANALOGPIN);                    //get reading from microphone

    if (sample < 1024) {                                  // toss out spurious readings
      if (sample > signalMax) {
        signalMax = sample;                           // save just the max levels
      }
      else if (sample < signalMin) {
        signalMin = sample;                           // save just the min levels
      }
    }
  }

  peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
  float Noise = map(peakToPeak,20,900,49.5,90);             //calibrate for deciBels
  Serial.print(Noise);
  Serial.println(" dB");

  return Noise;
  
}

float getVOC() {
	
  float AirQuality = gasSensor.getPPM();
  Serial.print(AirQuality);
  Serial.print(" ppm ");
  Serial.print(gasSensor.getRZero());
  Serial.print(" R0, ");
  Serial.print(gasSensor.getResistance());
  Serial.println(" RL");

  return AirQuality;
  
}

String getTimestamp() {
	
  timeClient.update();

  String weekDay = weekDays[timeClient.getDay()];
  
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  String currentMonthName = months[currentMonth-1];
  int currentYear = ptm->tm_year+1900;

  String FormattedDate = String(monthDay) + "/" + String(currentMonth) + "/" + String(currentYear);
  String NowStr = weekDay + " " + FormattedDate + " " + String(timeClient.getFormattedTime());
  return NowStr;
  
}

// --------------------------------------------------------------------------------
// HTTP/HTML section
// --------------------------------------------------------------------------------

void handle_OnConnect() {
	
  // HTTP request handle
  server.send(200, "text/html", SendHTML(String(getTemperature()), String(getTemperature2()), String(getFeelsLike()), String(getHumidity()), String(getUV()), String(getDUV()), String(getPressure()), String(getPressureSea()), String(getAltitude()), String(getAltitudeReal()), String(getNoise()), String(getVOC()), getTimestamp()));

}

void handle_NotFound() {
	
  server.send(404, "text/plain", "Not found");
  
}

String SendHTML(String Temperature, String Temperature2, String FeelsLike, String Humidity, String UV, String DUV, String Pressure, String PressureSea, String Altitude, String AltitudeReal, String Noise, String VOC, String NowStr) {

  // HTML Template
  // favicon with embedded Base64 string
  String HTMLStr = "<!DOCTYPE html> <html>\n";
  HTMLStr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  HTMLStr += "<title>ESP8266 Weather Station</title>\n";
  HTMLStr += "<link rel='shortcut icon' href='data:image/x-icon;base64,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AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA////ApKSSQ6nnUUarqhqKaGcUDannlg6pqFVOaOjXDKvr3ggtqSADv///wEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA==' />\n";
  HTMLStr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  HTMLStr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  HTMLStr += "p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
  HTMLStr += "</style>\n";
  HTMLStr += "</head>\n";
  HTMLStr += "<body>\n";
  HTMLStr += "<div id=\"webpage\">\n";
  HTMLStr += "<h1>ESP8266 Weather Station</h1>\n";
			 
  HTMLStr += "<p>";
  HTMLStr += NowStr;
  HTMLStr += "</p>\n";
			 
  HTMLStr += "<p>Temperature: ";
  HTMLStr += Temperature;
  HTMLStr += " &#8451;</p>\n";
			 
  HTMLStr += "<p>Temperature: ";
  HTMLStr += Temperature2;
  HTMLStr += " &#8451;</p>\n";
			 
  HTMLStr += "<p>Feels Like: ";
  HTMLStr += FeelsLike;
  HTMLStr += " &#8451;</p>\n";
			 
  HTMLStr += "<p>Humidity: ";
  HTMLStr += Humidity;
  HTMLStr += " %</p>\n";
			 
  HTMLStr += "<p>UV: ";
  HTMLStr += UV;
  HTMLStr += " mW/cm<sup>2</sup></p>\n";
			 
  HTMLStr += "<p>DUV: ";
  HTMLStr += DUV;
  HTMLStr += " index</p>\n";
			 
  HTMLStr += "<p>Pressure: ";
  HTMLStr += Pressure;
  HTMLStr += " kPa</p>\n";
			 
  HTMLStr += "<p>Pressure Sea Level: ";
  HTMLStr += PressureSea;
  HTMLStr += " kPa</p>\n";
			 
  HTMLStr += "<p>Altitude: ";
  HTMLStr += Altitude;
  HTMLStr += " meters</p>\n";
			 
  HTMLStr += "<p>Altitude Real: ";
  HTMLStr += AltitudeReal;
  HTMLStr += " meters</p>\n";
			 
  HTMLStr += "<p>Air Quality (VOC): ";
  HTMLStr += VOC;
  HTMLStr += "0 PPM</p>";
			 
  HTMLStr += "<p>Noise: ";
  HTMLStr += Noise;
  HTMLStr += "0 dB</p>";
			 
  HTMLStr += "</div>\n";
  HTMLStr += "</body>\n";
  HTMLStr += "</html>\n";

  return HTMLStr;
  
}