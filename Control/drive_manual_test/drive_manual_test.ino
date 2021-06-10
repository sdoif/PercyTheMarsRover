#include <WiFi.h>
#include <Ethernet.h>
#include <PubSubClient.h> // mqtt stuff

//setting up global variables/objects

#define visionIn 4
#define visionOut 2
#define driveIn 16
#define driveOut 17

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);
long lastMsg = 0;
char msg[90], fromDrive[90], fromVision[20];
char toVision[36], toCommand[54];
char toDrive[5] = {'0','x','0','0','0'};
int value = 0;
int bytein = 0;
int _index = 0;
String add;
String correct;
const char* serverip = "35.178.136.139"; // aws server ip


// setup always runs at the start
void setup() {
  
    Serial.println("Start");
    // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
    Serial1.begin(115200, SERIAL_8N1, visionIn, visionOut); // setting up uart port to speak with vision
    Serial2.begin(115200, SERIAL_8N1, driveIn, driveOut); // setting up uart port to speak with drive
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

    if(mqttclient.subscribe("speed")){
      Serial.println("Subscribed to speed");
    }else{
      Serial.println("Failed subscribing");
    }

    if(mqttclient.subscribe("mode")){
      Serial.println("Subscribed to mode");
    }else{
      Serial.println("Failed mode");
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
    WiFi.begin("Selin", "selinuygun"); // connects to wifi idk how to connect to imperial wifi as it needs authentication
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
  Serial.print(topic);
  Serial.println();
  String _topic = String(topic);
  //send whatever direction command we receive as its already good!
  if(_topic=="test"){
       Serial.print(messageTemp);
  }else if (_topic=="mode"){
      toDrive[0] = messageTemp[0];
      Serial2.print("c" + String(toDrive));
      Serial.print("c" + String(toDrive));
  }else if(_topic=="direction"){
      toDrive[1] = messageTemp[0];
      Serial2.print("c" + String(toDrive));  
      Serial.print("c" + String(toDrive));
  }else if(_topic=="speed"){
    Serial.println("HERE --> speed");
     toDrive[2] = messageTemp[0];
     toDrive[3] = messageTemp[1];
     toDrive[4] = messageTemp[2];
     Serial2.print("c" + String(toDrive));
     Serial.print("c" + String(toDrive));
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
  
  if(!mqttclient.connected()) {
    reconnect();
  }
  
  mqttclient.loop();

  if(Serial2.available() > 89){

      for(int i = 0; i < 90 ; i++){
        bytein = Serial2.read();
        msg[i] = char(bytein);
      }

    add = "";

    for(int i = 0; i < 2; i++){
      for(int i = 0; i<90; i++){
        add = add + msg[i];
      }
    }

    Serial.println("add = " + add);
    
    _index = add.indexOf('c');
      
    correct = "";
    for(int i = _index; i < _index + 90; i++){
      correct = correct + add[i];
    }

    Serial.println("correct = " + correct);

    for(int i = 1; i < 54; i++){
      toCommand[i] = correct[i];
    }
    for(int i = 54; i < 80; i++){
      toVision[i-54] = correct[i];
    }

    
    for(int i = 1; i<54; i++){
      Serial.print(toCommand[i]);
    }
    Serial.println();
    byte buffer[54];
    for(int i = 0; i < 54; i++){
      buffer[i] = byte(toCommand[i]);
    }
    mqttclient.publish("drive", buffer, 54);
    Serial1.print(toVision);
    Serial.print("toVision = "+String(toVision));
  }


  if(Serial1.available()){
    int k = 0;
    while(Serial1.available()){
      bytein = Serial1.read();
      fromVision[k] = char(bytein);
      k++;
    }
    Serial.print("Received from vision: ");
    Serial.println(fromVision);
    Serial2.print("v" + String(fromVision));
    
  }

  if(Serial.available()){
    int i = 0;
    while(Serial.available()){
      bytein = Serial.read();
      msg[i] = char(bytein);
      i++;
    }
    mqttclient.publish("test", msg);
    clearmsg();
  }

}
