#define RXD2 16
#define TXD2 17

#define RXD1 4
#define TXD1 2

int bytein = 0;
char msg[50];
int i = 0;

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop(){
    Serial.println("Data received: ");
    i = 0;
    while(Serial2.available()){
      bytein = Serial2.read();
      msg[i] = char(bytein);
      i++;
    }
    Serial.println(msg);
    delay(100);
}
