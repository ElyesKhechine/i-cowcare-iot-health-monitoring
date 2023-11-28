#include <AsyncTCP.h>
#include <AsyncMqttClient.h>
#include <Arduino.h>
#include <C:\Users\Elyes\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.9\libraries\WiFi\src\WiFi.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <lis2dh12.h>


extern "C" {
  #include "FreeRTOS/FreeRTOS.h"
  #include "FreeRTOS/timers.h"
}

#define WIFI_SSID "REPLACE_WITH_YOUR_SSID"
#define WIFI_PASSWORD "REPLACE_WITH_YOUR_PASSWORD"

// BROKER
#define MQTT_HOST IPAddress(192, 168, 1, 99) //MQTT BROKER IP ADDRESS
#define MQTT_PORT 1883
#define BROKER_USER "icowcare"
#define BROKER_PASS "icowcare2023"

//MQTT Subscribe Topics
#define MQTT_SUB_DIGITAL "esp32/digital/#"


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


void connectToWifi() {
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  Serial.println(SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  // we can even make the ESP32 to sleep
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  while (!mqttClient.connected()) {
     Serial.println("Reconnecting to MQTT Broker..");
     String clientId = "ESP32Client-";
     clientId += String(random(0xffff), HEX);
    
     if (mqttClient.connect(clientId.c_str())) {
       Serial.println("Connected.");
       // subscribe to topic 
       mqttClient.subscribe("esp32/output");     
     }
     else{
       Serial.print("failed, rc=");
       Serial.print(mqttClient.state());
       Serial.println(" try again in 5 seconds");
       // Wait 5 seconds before retrying
       delay(5000);
    }
 }
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  //Subscribe to topics
  // Subscribe to topic MQTT_SUB_DIGITAL when it connects to the broker
  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_DIGITAL, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttMessage(char* topic, char* message, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Do whatever you want when you receive a message
  
  // Save the message in a variable
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  
  String receivedMessage;
  for (int i = 0; i < len; i++) {
    Serial.println((char)message[i]);
    receivedMessage += (char)message[i];
  }
  // Save topic in a String variable
  String receivedTopic = String(topic);  
  Serial.print("Received Topic: ");
  Serial.println(receivedTopic);
  
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

///temperature sensor Adafruit_MLX90614
//Defining the object for mlx
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
long lastime = 0;
void temperature(void){
   float tAmb, tObj;
   char data_tAmb[10];
   char data_tObj[10];
   tAmb = mlx.readAmbientTempC();
   tObj = mlx.readObjectTempC();
   
   dtostrf(tAmb, 5, 2, data_tAmb);  // Convert tAmb to string with 5 digits, 2 decimal places
   dtostrf(tObj, 5, 2, data_tObj);  // Convert tObj to string with 5 digits, 2 decimal places

   Serial.print("Ambient = ");
   Serial.println(data_tAmb);
   Serial.print("*C\tObject = ");
   Serial.print(data_tObj);
   Serial.println("*C");
   mqttClient.publish("esp32/temp_amb", data_tAmb);
   mqttClient.publish("esp32/temp_obj", data_tObj);   
}


//Accelerometer LIS2DH
LIS2DH accel(A0); //Accelerometer
void acceleration(void)
{
  accel.init();
  int16_t x, y, z;
  char data_accel_x[10];
  char data_accel_y[10];
  char data_accel_z[10];
  delay(100);
  accel.getMotion(&x, &y, &z);

  dtostrf(x, 5, 0, data_accel_x);  // Convert x to string with 5 digits, 0 decimal places
  dtostrf(y, 5, 0, data_accel_y);  // Convert y to string with 5 digits, 0 decimal places
  dtostrf(z, 5, 0, data_accel_z);  // Convert z to string with 5 digits, 0 decimal places

  Serial.print("Acceleration x: "); //print acceleration
  Serial.print(x);
  Serial.print(" mg \ty: ");
  Serial.print(y);
  Serial.print(" mg \tz: ");
  Serial.print(z);
  Serial.println(" mg");

  mqttClient.publish("esp32/accel_x", data_accel_x);
  mqttClient.publish("esp32/accel_y", data_accel_y);
  mqttClient.publish("esp32/accel_z", data_accel_z);
}


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(BROKER_USER, BROKER_PASS);

  connectToWifi();
  mlx.begin();
  if (!mlx.begin(0x76)) {
    Serial.println("Could not find a valid MLX90614 temperature sensor sensor, check wiring!");
    while (1);
  }
}


void loop() {
  if (!mqttClient.connected())
     connectToMqtt();
    mqttClient.loop();
   
   // Publishing data to MQTT
   long now = millis();
   if(now - lastime > 10000) {
     Serial.println("Publishing data..");
     temperature();
     acceleration();
     lastime = now;
 }
}
