#include <WiFi.h>
#include <Ethernet.h>
#include <PubSubClient.h>

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
int count;
int endVisionMessage = 0;
int endDriveMessage = 0;
int visionIt = 0;
int driveIt = 0;
char add[3];
const char* serverip = "18.134.3.99"; // aws server ip

void setup() {
  
    Serial.println("Start");
    Serial1.begin(115200, SERIAL_8N1, visionIn, visionOut); // setting up uart port to speak with vision
    Serial2.begin(115200, SERIAL_8N1, driveIn, driveOut); // setting up uart port to speak with drive
    Serial.begin(115200); 
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
    if(mqttclient.subscribe("direction")){
      Serial.println("Subscribed to direction");
    }
    if(mqttclient.subscribe("speed")){
      Serial.println("Subscribed to speed");
    }
    if(mqttclient.subscribe("mode")){
      Serial.println("Subscribed to mode");
    }

}

int setupwifi()
{
    WiFi.begin("Selin", "selinuygun");
    Serial.print("Waiting for WiFi... "); 
    while(WiFi.status() != WL_CONNECTED) { 
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

  String _topic = String(topic);
  if (_topic == "mode"){
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
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttclient.connect("esp32")) {
      Serial.println("Connected to Broker");
      Serial.println("Subscribing to Topics");
      mqttclient.subscribe("direction");
      mqttclient.subscribe("speed");
      mqttclient.subscribe("test");
      
    }else{
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 3 seconds");
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
      
      Serial.print("From Drive: ");
      Serial.println(fromDrive);
      
      if(fromDrive[0] == 'c'){
        mqttclient.publish("drive", fromDrive, 50);
      }else if(fromDrive[0] == 'v'){
        Serial.print(fromDrive);
        //Serial1.print(fromDrive[0]);
        Serial1.print(fromDrive[1]);
        //Serial1.print(fromDrive[2]);

      }else if(fromDrive[0] == 'b'){
        mqttclient.publish("ball", fromDrive, 50);
      }
      clearFromDrive();
      endDriveMessage = 0;
    }
  }

  if(Serial1.available()){
    char readChar;  
    while(Serial1.available()){
      readChar = Serial1.read();
      Serial.print("Char from Vision ");
      Serial.println(readChar);
      if(readChar == '!'){
        endVisionMessage = 1;
        visionIt = 0;
        break;
      }
      readVision[visionIt] = readChar;
      visionIt++;
    }

    if(endVisionMessage){
      

      if(readVision[1] == 'c'){
        mqttclient.publish("vision", readVision, 20);
      }else if((readVision[1] == 'v')||(readVision[0] == 'v')){
        for(int i = 0; i < 20; i++){
          if(readVision[i] == 'v'){
            add[0] = readVision[i];
            count = 1;
          }else if( ((readVision[i] == '0') || (readVision[i] == '1')) && (count == 1) ){
            add[1] = readVision[i];
            count = 2;
          }else if(((readVision[i] == 'g')||(readVision[i] == 's')||(readVision[i] == 'r')||(readVision[i] == 'l'))&&(count == 2)){
            add[2] = readVision[i];
            Serial2.print(add);
            Serial.println("from vision ="+String(add));
            break;
          }else{
            count = 0;
          }
        }
        
        //Serial2.print(readVision); 
      }

      clearReadVision();
      endVisionMessage = 0;
    } 
  }
}
