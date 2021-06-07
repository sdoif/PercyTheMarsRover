
int c_new = 0;
int v_new = 0;
char _mode;
char c[6];
char v[2];

void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() > 0){
    char _modenew = Serial.read();
    if(_mode!=_modenew){
      _mode=_modenew;
    }
    if(_mode == 'c'){
      _mode = NULL;
      for(int i = 0; i<6; i++){
        c_new = Serial.read();
        c[i] = char(c_new);
      }
    }else if(_mode == 'v'){
       _mode = NULL;
      for(int i = 0; i<2; i++){
        v_new = Serial.read();
        v[i] = char(v_new);
      }
    }
  }
    Serial.println("Data received: ");
    Serial.print("c[] =");
    Serial.println(c);
    Serial.print("v[] =");    
    Serial.println(v);

}
