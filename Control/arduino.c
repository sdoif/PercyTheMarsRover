/*

Starting code that will connect to wifi and establish mqtt connection, subscribe to a thread and publish a message to another thread.

*/

#include <WiFi.h>
#include <Ethernet.h>
#include <PubSubClient.h> // mqtt stuff

//setting up global variables/objects

#define visionIn 5
#define visionOut 6
#define driveIn 7
#define driveOut 8

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);
long lastMsg = 0;
char msg[20], fromdrive[43], fromvision[20];
char toDrive[6] = "0x000";
int value = 0;
int bytein = 0;
const char* serverip = "3.80.119.99"; // aws server ip


// setup always runs at the start
void setup() {
  
    Serial.println("Start");
    // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
    //Serial2.begin(115200, SERIAL_8N1, visionIn, visionOut); // setting up uart port to speak with vision
    //Serial1.begin(115200, SERIAL_8N1, driveIn, driveOut); // setting up uart port to speak with drive
    Serial.begin(115200); // sets baud rate
    delay(10);
    Serial.println("Connecting to wifi");

    if(setupwifi()<0){
      Serial.print("Error connecting to wifi");
    }
    
    Serial.println("Setting up mqttclient");
    mqttclient.setServer(serverip, 1883);   
    mqttclient.setCallback(callback);
    
    if(!mqttclient.connect("esp32")){
      Serial.println("Client connection failed");
    }
    delay(1000);
    if(mqttclient.subscribe("direction")){
      Serial.println("Subscribed to direction");
    }else{
      Serial.println("Failed subscribing");
    }
    
    if(mqttclient.publish("test", "hello from esp32")){
      Serial.println("Message sent");
    }else{
      Serial.println("Message failed to send");
    }

}

int setupwifi()
{
    // We start by connecting to a WiFi network
    WiFi.begin("Kai", "kai12456"); // connects to wifi idk how to connect to imperial wifi as it needs authentication
    Serial.print("Waiting for WiFi... "); 
    while(WiFi.status() != WL_CONNECTED) { // this just tries to connect to wifi i guess
        Serial.print(".");
        delay(500);
    }
    Serial.println("WiFi connected");
    delay(10); 
    return 0;
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  //send whatever direction command we receive as its already good!
  if(topic == "direction"){
      toDrive[1] = messageTemp[0];
      Serial2.print("c" + String(toDrive);  
  }else if(topic == "speed"){
      toDrive[2] = messageTemp[0];
      toDrive[3] = messageTemp[1];
      toDrive[4] = messageTemp[2];
      Serial2.print("c" + String(toDrive));
  }else if(topic == "test"){
    Serial.print(messageTemp);
  }else if (topic == "mode"){
      toDrive[0] = messageTemp[0];
      Serial2.print("c" + messageTemp);
  }
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect("esp32")) {
      Serial.println("Connected to Broker");
      // Subscribe
      Serial.println("Subscribing to Topics");
      mqttclient.subscribe("direction");
      mqttclient.subscribe("speed");
      mqttclient.subscribe("test");
      
    }else{
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 3 seconds");
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}

void clearmsg(){
  for(int i = 0; i<20; i++){
    msg[i] = NULL;
  }
}

void loop() {

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Reconnecting to wifi");
    setupwifi();
  }
  
  while(!mqttclient.connected()) {
    reconnect();
  }
  
  mqttclient.loop();

  int i = 0;
  int v = 0;
  if(Serial.available()){
    v = 1;
    i = 0;
    while(Serial.available()){
      bytein = Serial.read();
      msg[i] = char(bytein);
      i++;
    }
  }


  if(v){
    mqttclient.publish("test", msg);
    clearmsg();
  }
  delay(1000);

}