// information received is 9 bits
// rx[8] and rx[7] for directions(00 for forward 01 for turn right 10 for turn left and 11 for backward)
// rx[8] is used to determine if other bits are going to be used for control signals or for numerical data transfer(1 when numerical data)
// rx[7] is used to determine if numerical data is angle or distance(1 for angle)(used when rx[8]=1)
// rx[7] is STOP signal which is high whenever a target is seen (used when rx[8]=0 and rx[7]=0)
// rx[6] is STOP_and_Change signal which is high while moving to a target if the rover changes direction(used when rx[8]=0 and rx[7]=0 and rx[6]=0)
// rx[5] is Stop signal if the target is alligned after rx[6] action

//autonomous mode
// received data is numerical values
//String receiveddata(String rx){
//  char rx8 = rx.charAt(9);
//  char rx7 = rx.charAt(8);
//  char rx6 = rx.charAt(7);
//  char rx5 = rx.charAt(6);
//  char rx4 = rx.charAt(5);
//  char rx3 = rx.charAt(4);
//  char rx2 = rx.charAt(3);
//  char rx1 = rx.charAt(2);
//  char rx0 = rx.charAt(1);
//  char rxall[9] = {rx0, rx1, rx2, rx3, rx4, rx5, rx6, rx7, rx8};
//  return rxall;
//}
 int getdistance(String rx){ //input is actually rx[5:0] i don't know how to write that as input of this function
  //receiveddata(rx);
  int distance;
  if(rx.charAt(0)=='1' & rx.charAt(1)=='0'){
    rx.remove(0,2);
    int distance = rx.toInt();
    return distance;
  }else{
   return 0;
  }
 }
 
  int getangle(String rx){ //another option is this, having no inputs to the function
  int angle;
  if(rx.charAt(0)=='1' & rx.charAt(1)=='1'){
    rx.remove(0,2);
    int angle = rx.toInt(); // don't forget twos complement so actually the angle range is going to be 31 to -31
    return angle;
  }else{
    return 360;
  }
  }
  
 int xcoordinatefinder(){
    getdistance(distance);
    getangle();
    total_x = distance*cos(angle) + total_x;
    return total_x;
  }
  
  int ycoordinatefinder(int distance, int angle){
    getdistance(distance);
    getangle();
    total_y = distance*sin(angle) + total_y;
    return total_y;
  }
void reach_forward(){
    if(rx[6] == LOW){
    DIRRstate = HIGH;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
    }else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
    getangle();
      if(angle>0){
        turn_right();
        reach_forward();
      }
      else if(angle<0){
        turn_left();
        reach_ forward();
      }
  }
  }
  void turn_right(){
   if(rx[5]== LOW){
    DIRRstate = HIGH;
    DIRLstate = HIGH;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
  }else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }}
  
  void backward(){
    DIRRstate = LOW;
    DIRLstate = HIGH;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
  }
  
  void turn_left(){
    if(rx[5] == LOW){
    DIRRstate = LOW;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
  }else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }}
  void brake(){
    DIRRstate = HIGH;
    DIRLstate = LOW;
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }
//  void scan(bool STOP, bool scandegree){
//    //rotating clockwise
//    while(STOP == LOW || scandegree == LOW){
//      vref = 1.8;
//    DIRRstate = HIGH;
//    DIRLstate = HIGH; 
//    digitalWrite(pwmr, HIGH);
//    digitalWrite(pwml, HIGH);
//    }while (STOP == HIGH || scandegree == HIGH) {
//    digitalWrite(pwmr, LOW);
//    digitalWrite(pwml, LOW);
//    }while(STOP == HIGH && scandegree == HIGH){
//      
//      scan();
//      }
//     }
   
    
  void manual_forward(){
    if(rx[8]==1 && rx[7]==0 && rx[6]==0 && rx[5]==0){
    DIRRstate = HIGH;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
    }
    else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }
  void manual_right(){
    if(rx[8]==0 && rx[7]==1 && rx[6]==0 && rx[5]==0){
    DIRRstate = HIGH;
    DIRLstate = HIGH;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
  }else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }}
  
  void backward(){
    if(rx[8]==0 && rx[7]==0 && rx[6]==1 && rx[5]==0){
    DIRRstate = LOW;
    DIRLstate = HIGH;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
  }else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }}
  
  void manual_left(){
    if(rx[8]==0 && rx[7]==0 && rx[6]==0 && rx[5]==1){
    DIRRstate = LOW;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH);
  }else{
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }}
  void brake(){
    DIRRstate = HIGH;
    DIRLstate = LOW;
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
  }
//  struct coordinates{
//    int x;
//    int y;
//  }
//  struct coordinatefinder(int distance, int angle){
//    total_x = distance*cos(angle) + total_x; // eklencek item var
//    total_y = distance*sin(angle) + total_y;
//    coordinates.x = total_x;
//    coordinates.y = total_y;
//    return coordinates;
//  }
  
  

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
