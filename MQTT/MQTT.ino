#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <Wire.h>
#include <DFRobot_LIS2DH12.h>
#include "time.h"
#include "mqtt.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_MLX90614.h>

/****** WiFi Connection Details *******/
const char* ssid =  "dlink_aerobotix" ; //input WiFi SSID
const char* password = "aerobotix2022" ;//input WiFi Password

int timestamp = 1;

bool online = false ; // set true if using HiveMQTT (online) and false of using Mosquitto MQTT (local broker)
// NTP server to request epoch time (required for timestamp)
const char* ntpServer = "pool.ntp.org";

/******* MQTT Broker Connection Details *******/
//condition ? expression1 : expression2;
const char* mqtt_server = "10.13.0.204"   ; //input local or online MQTT server IP address
const char* mqtt_username = "ICowCare";  // input MQTT username
const char* mqtt_password = "ICowCare1"; // input MQTT password
const int mqtt_port = 1883;              // input MQTT port

WiFiClient espClient; //if online change with WiFiClientSecure espClient;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE]; // MQTT data transfer message (buffer size : 50)

DFRobot_LIS2DH12 LIS; //Accelerometer
int16_t x, y, z;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();  //Initialize temperature sensor object

/**** Application Initialisation Function******/
void setup() {
  Wire.begin();
  Serial.begin(115200);

  while (!Serial) delay(1);
  //connect to Wifi
  setup_wifi();
  configTime(0, 0, ntpServer);
  //connect to MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //connect to accelerometer
  while (LIS.init(LIS2DH12_RANGE_2GA) == -1) { //Equipment connection exception or I2C address error
    Serial.println("No I2C devices found");
    delay(500) ;
  }
  while (!Serial);

  //connect to temperature sensor
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  // enable these line and the the "certificate" code for secure connection
  /*  #ifdef ESP8266
      espClient.setInsecure();
    #else
      espClient.setCACert(root_ca);
    #endif*/

}

/******** Main Function *************/
void loop() {
  if (!client.connected()) reconnect(); // check if client is connected
  client.loop();

  //Get current timestamp
  timestamp = getTime();
  Serial.print ("time: ");
  Serial.println (timestamp);

  //Get accelerometer data
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  Serial.print("Acceleration x: "); //print acceleration
  Serial.print(x);
  Serial.print(" mg \ty: ");
  Serial.print(y);
  Serial.print(" mg \tz: ");
  Serial.print(z);
  Serial.println(" mg");

  //  JSON SERIALIZATION
  DynamicJsonDocument doc(128);

  //doc["deviceId"] = "Cow No 1";
  doc["accel_x"] = x;
  doc["accel_y"] = y ;
  doc["accel_z"] = z;
  //doc["timestamp"] = timestamp;

  String jsonString;
  serializeJson(doc, jsonString);

  // publish accelerometer data to topic test/accel
  publishMessage("test/accel", jsonString, true);

  // prepare different sensor data to be sent via Wifi/MQTT
  String xacc = String(x);
  String yacc = String(y);
  String zacc = String(z);
  String timestamps = String(timestamp);
  String tempAmb = String(mlx.readAmbientTempC()) ;
  String tempObj = String(mlx.readObjectTempC()) ;

  Serial.print("Ambient Temperature is");
  Serial.print(mlx.readAmbientTempC());
  Serial.println("°C");
  Serial.print("Object Temperature is");
  Serial.print(mlx.readObjectTempC());
  Serial.println("°C");

  // publish data to different topics to be used in database, dashboard ...
  publishMessage("test/cowid", "Cow No 1", true);
  publishMessage("test/x", xacc, true);
  publishMessage("test/y", yacc, true);
  publishMessage("test/z", zacc, true);
  publishMessage("test/tempamb", tempAmb, true);
  publishMessage("test/tempobj", tempObj, true);
  publishMessage("test/timestamp", timestamps, true);
  delay(100);  // delay to set data rate to 10Hz (10 data feedbacks per second) 
}
