
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if(Serial1.available() > 0){
    Serial.print("Data received: ");
    Serial.println(Serial1.readString());
    delay(200);
  }
}
