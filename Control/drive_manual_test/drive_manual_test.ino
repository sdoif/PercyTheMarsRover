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
char msg[90], fromDrive[50], toVision[36], toCommand[44], ballCoordinates[50], readVision[20];
char toDrive[5] = {'0','x','0','0','0'};
int bytein = 0;
int _index = 0;
int endVisionMessage = 0;
int endDriveMessage = 0;
int visionIt = 0;
int driveIt = 0;
String add, correct;
const char* serverip = "18.134.3.99"; // aws server ip


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

void callback(char* topic, byte* message, unsigned int _length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  for (int i = 0; i < _length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  Serial.print(topic);
  Serial.println();
  String _topic = String(topic);
  //send whatever direction command we receive as its already good!
  if(_topic == "test"){
       Serial.print(messageTemp);
  }else if (_topic == "mode"){
      toDrive[0] = messageTemp[0];
      Serial2.print("c" + String(toDrive));
      Serial.print("c" + String(toDrive));
  }else if(_topic == "direction"){
      toDrive[1] = messageTemp[0];
      Serial2.print("c" + String(toDrive));  
      Serial.print("c" + String(toDrive));
  }else if(_topic == "speed"){
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

void clearReadVision(){
  for(int i = 0; i < 20; i++)
  readVision[i] = NULL;
}

void clearFromDrive(){
  for(int i = 0; i < 50; i++){
    fromDrive[i] = NULL;
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

  if(Serial2.available()){
    char readchar;
    while(Serial2.available()){
      readchar = Serial2.read();
      if(readchar == '!'){
        endDriveMessage = 1;
        driveIt = 0;
        break;
      }
      fromDrive[driveIt] = readchar;
      driveIt++;
    }

    if(endDriveMessage){

      if(fromDrive[0] == 'c'){
        mqttclient.publish("drive", fromDrive, 50);
      }else if(fromDrive[0] == 'v'){
        char toVision2[3];
        toVision2[0] = fromDrive[0];
        toVision2[1] = fromDrive[1];
        toVision2[2] = fromDrive[2];
        Serial1.print(toVision2);
      }else if(fromDrive[0] == 'b'){
        mqttclient.publish("ball", fromDrive, 50);
      }
      clearFromDrive();
      endDriveMessage = 0;
    }
  }

  if(Serial1.available()){
    char readchar;
    while(Serial1.available()){
      readchar = Serial1.read();
      if(readchar == '!'){
        endVisionMessage = 1;
        visionIt = 0;
        break;
      }
      readVision[visionIt] = readchar;
      visionIt++;
    }

    if(endVisionMessage){

      Serial.print("Received from vision: ");
      Serial.println(readVision);

      if(readVision[0] == 'c'){
        mqttclient.publish("vision", readVision, 20);
      }else if(readVision[0] == 'v'){
        Serial2.print(readVision);
      }

      clearReadVision();
      endVisionMessage = 0;
    } 
  }

  if(Serial.available()){
    int i = 0;
    char readChar;
    while(Serial.available()){
      readChar = Serial.read();
      msg[i] = readChar;
      i++;
    }
    mqttclient.publish("test", msg);
    clearmsg();
  }

}
