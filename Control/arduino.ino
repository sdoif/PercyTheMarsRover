/*

Starting code that will connect to wifi and establish mqtt connection, subscribe to a thread and publish a message to another thread.

*/

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
char msg[20], fromDrive[43], fromVision[20];
char toVision[13], toCommand[28];
char toDrive[5] = {'0','x','0','0','0'};
int value = 0;
int bytein = 0;
const char* serverip = "3.91.160.250"; // aws server ip


// setup always runs at the start
void setup() {
  
    Serial.println("Start");
    // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
    Serial1.begin(9600, SERIAL_8N1, visionIn, visionOut); // setting up uart port to speak with vision
    Serial2.begin(9600, SERIAL_8N1, driveIn, driveOut); // setting up uart port to speak with drive
    Serial.begin(115200); // sets baud rate
    delay(10);
    Serial.println("Connecting to wifi");

    if(setupwifi()<0){
      Serial.print("Error connecting to wifi");
    }
    
    Serial.println("Setting up mqttclient");
    mqttclient.setServer(serverip, 1883);   
    mqttclient.setCallback(callback);

    mqttconnect();
    
    delay(1000);
    


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

  String _topic = String(topic);
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
     toDrive[2] = messageTemp[0];
     toDrive[3] = messageTemp[1];
     toDrive[4] = messageTemp[2];
     Serial2.print("c" + String(toDrive));
     Serial.print("c" + String(toDrive));
  }
  
}

void mqttconnect() {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect("esp32")) {
      Serial.println("Connected to Broker");
      Serial.println("Subscribing to Topics");
      if( (mqttclient.subscribe("direction")) && (mqttclient.subscribe("mode")) && (mqttclient.subscribe("speed")) ){
        Serial.println("Subscribed to topics");
      }else{
        Serial.println("Failed to subscribe to all topics");
      } 

      if(mqttclient.publish("test", "hello from esp32")){
        Serial.println("Test message sent");
      }else{
        Serial.println("Test message failed to send");
      }
      
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

  Serial.println("Start of loop");
  
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Reconnecting to wifi");
    setupwifi();
  }else if(!mqttclient.connected()){
    mqttconnect();
  }

  
  mqttclient.loop();

  Serial.println("b4 reading serials");

  if(Serial2.available()){
    int j = 0;
    while(Serial2.available()){
      bytein = Serial2.read();
      fromDrive[j] = char(bytein);
      j++;
    }

    for(int i = 1; i < 29; i++){
      toCommand[i] = fromDrive[i];
    }
    for(int i = 30; i < 43; i++){
      toVision[i-30] = fromDrive[i];
    }
    
    mqttclient.publish("drive", toCommand);
    Serial1.print(toVision);
  }

  Serial.println("after drive reaad");

  
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
  
  //v is at index 29

  Serial.println("after vision read");

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


  delay(1000);

}