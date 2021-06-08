#define RXD2 16
#define TXD2 17

#define RXD1 4
#define TXD1 2

int bytein = 0;
char msg[43];
//char str[6];

void clearmsg(){
  for(int i =0; i<43; i++){
    msg[i] = NULL;
  }
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop(){
    if(Serial2.available() > 42){
      for(int i = 0; i<43; i++){
      bytein = Serial2.read();
      msg[i] = char(bytein);
      }
    }
    Serial.println("Data received: ");
    Serial.println(msg);
    //clearmsg();
    delay(80);
    
  if(Serial.available() > 0){
//      for(int i=0; i<6; i++){
//        bytein = Serial.read();
//        str[i] = char(bytein);
//      }
//  }
    String str = Serial.readString();
    Serial2.println(str);
    Serial.print("Data sent: ");    
    Serial.println(str);
    Serial.println();
  }
}
