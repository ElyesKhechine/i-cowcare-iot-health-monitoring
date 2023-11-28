#include <Arduino.h>
#include <PubSubClient.h>
#include <C:\Users\Elyes\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.9\libraries\WiFi\src\WiFi.h>
#include <WiFiClient.h>
#include <lis2dh12.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

IPAddress mqttServer(192,168,1,107);
//const char* mqtt_server = "192.168.1.107";
const char *SSID = "Orange-86BF";
const char *PWD = "Orange-86BF2023*";

#define BROKER_USER "summerschool"
#define BROKER_PASS "summerschool2023"
 
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String receivedMessage;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    receivedMessage += (char)message[i];
  }
  Serial.println();
    // Feel free to add more if statements to control more GPIOs with MQTT
}

WiFiClient wifiClient = WiFiClient();
PubSubClient mqttClient(mqttServer, 1883, callback, wifiClient);
 
void connectToWifi() {
 delay(10);
 Serial.print("Connecting to ");
 Serial.println(SSID);
  WiFi.begin(SSID, PWD);
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
 Serial.println("Connecting to MQTT Broker...");
 String clientId = "ESP32Client-";
 clientId += String(random(0xffff), HEX);
 if (mqttClient.connect(clientId.c_str(), BROKER_USER, BROKER_PASS)) {
   Serial.println("Connected.");
    //subscribe to topic  
   //mqttClient.subscribe("esp32/output");    
 }
 else{
   Serial.print("failed, rc=");
   Serial.print(mqttClient.state());
   Serial.println(" try again in 5 seconds");
   // Wait 5 seconds before retrying
   delay(5000);
}
 
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
 connectToWifi();
 //mlx.begin();
 //if (!mlx.begin(0x76)) {
   // Serial.println("Could not find a valid MLX90614 sensor, check wiring!");
   // while (1);
 // }
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
     lastime = now;
 }
}
