
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    Serial.print("Data received: ");
    Serial.println(Serial1.readString());
    delay(200);
}
