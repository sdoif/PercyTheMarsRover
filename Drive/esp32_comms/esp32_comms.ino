#define RXD2 16
#define TXD2 17

#define RXD1 4
#define TXD1 2

int bytein = 0;
char msg[45];

void clearmsg(){
  for(int i =0; i<45; i++){
    msg[i] = NULL;
  }
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop(){
    if(Serial2.available() > 44 ){
      for(int i = 0; i<45; i++){
      bytein = Serial2.read();
      msg[i] = char(bytein);
      }
    }
    Serial.println("Data received: ");
    Serial.println(msg);
    //clearmsg();
    delay(80);
}
