#define RXD2 16
#define TXD2 17

#define RXD1 4
#define TXD1 2


void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop(){
    Serial.println("Data sent");
    Serial2.println("100001011");
    delay(1500);
    //delayMicroseconds(10);
}
