#define RXD2 16
#define TXD2 17

#define RXD1 4
#define TXD1 2

int _index = 0;
int bytein = 0;
char msg[70];
String add;
String correct;
//char str[6];

void clearmsg(){
  for(int i =0; i<70; i++){
    msg[i] = NULL;
  }
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop(){
    if(Serial2.available() > 69){
      for(int i = 0; i<70; i++){
      bytein = Serial2.read();
      msg[i] = char(bytein);
      }
    }
  add = "";
  for(int i =0; i<2; i++){
    for(int i = 0; i<70; i++){
      add = add + msg[i];
    }
  }
    
    _index = add.indexOf('c');
    
  correct = "";
  for(int i = _index; i<_index+70; i++){
    correct = correct + add[i];
  }
    Serial.println("Data received: ");
    Serial.println(msg);
    Serial.println(add);
    Serial.println("index of c = "+String(_index));
    Serial.println("correct = "+String(correct));
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
