#include "SPI.h"
#include "functions.h"
#include <Wire.h>
#include <Console.h>

void setup() 
{
  
    //*** Motor Pins Defining ***//
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  pinMode(pwmr, OUTPUT);
  pinMode(pwml, OUTPUT);
  digitalWrite(pwmr, HIGH);       //setting right motor speed at maximum
  digitalWrite(pwml, HIGH);       //setting left motor speed at maximum
  //*********//

  //Basic pin setups
  
   //disable all interrupts
  noInterrupts();
  //pinMode(13, OUTPUT);  //Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); //Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  
  TCA0.SINGLE.PER = 999; //255;
  TCA0.SINGLE.CMP1 = 999; //255; 
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  
  pinMode(6, OUTPUT);
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  
  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  //pinMode(PIN_SCK,OUTPUT);
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(38400);
  Serial1.begin(9600);

  if(mousecam_init()==-1)
  {
    Serial.println("Mouse cam failed to init");
    while(1);
  }  
  mousecam_write_reg(11, 7);
  mousecam_write_reg(25, 126);
  mousecam_write_reg(26, 14);

}


byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

void loop(){
  
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt
    
   //digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    if (Boost_mode){
      if (CL_mode) { //Closed Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
      }else{ // Open Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
      }
    }else{      
      if (CL_mode) { // Closed Loop Buck
          // The closed loop path has a voltage controller cascaded with a current controller. The voltage controller
          // creates a current demand based upon the voltage error. This demand is saturated to give current limiting.
          // The current loop then gives a duty cycle demand based upon the error between demanded current and measured
          // current
          current_limit = 3; // Buck has a higher current limit
          ev = vref - vb;  //voltage error at this time
          cv=pidv(ev);  //voltage pid
          cv=saturation(cv, current_limit, 0); //current demand saturation
          ei=cv-iL; //current error
          closed_loop=pidi(ei);  //current pid
          closed_loop=saturation(closed_loop,0.99,0.01);  //duty_cycle saturation
          pwm_modulate(closed_loop); //pwm modulation
      }else{ // Open Loop Buck
          current_limit = 3; // Buck has a higher current limit
          oc = iL-current_limit; // Calculate the difference between current measurement and current limit
          if ( oc > 0) {
            open_loop=open_loop-0.001; // We are above the current limit so less duty cycle
          } else {
            open_loop=open_loop+0.001; // We are below the current limit so more duty cycle
          }
          open_loop=saturation(open_loop,dutyref,0.02); // saturate the duty cycle at the reference or a min of 0.01
          pwm_modulate(open_loop); // and send it out
      }
    }
    // closed loop control path

    //digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
  }
  
  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.
 
  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for(int i=0; i<md.squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');

  //delay(100);

  distance_x = md.dx; //convTwosComp(md.dx);
  distance_y = md.dy; //convTwosComp(md.dy);
  
  total_x1 = (total_x1 + distance_x);
  total_y1 = (total_y1 + distance_y);
  
  total_x = 10*total_x1/157; //Conversion from counts per inch to mm (400 counts per inch)
  total_y = 10*total_y1/157; //Conversion from counts per inch to mm (400 counts per inch)

//total_x1_distance = total_x1_distance + abs(distance_x);
//total_y1_distance = total_y1_distance + abs(distance_y);
//
//total_x_distance = 10*total_x1_distance/157; //Conversion from counts per inch to mm (400 counts per inch)
//total_y_distance = 10*total_y1_distance/157; //Conversion from counts per inch to mm (400 counts per inch)
//
//
//Serial.println("Distance_x_total = " + String(total_x_distance));
//Serial.println("Distance_y_total = " + String(total_y_distance));
//Serial.print('\n');

  //counter stuff
  //min threshold for prev_val needs to be tested based on the maximum speed of the rover
  if(((200<=prev_val_y)&&(prev_val_y<=208))&&((-208<=total_y)&&(total_y<=-200))){
    counter_y++;
  }else if(((-200>=prev_val_y)&&(prev_val_y>=-208))&&((200<=total_y)&&(total_y<=208))){
     counter_y--;
  }
  if(counter_y!=0){
    if(((0<=prev_val_y)&&(prev_val_y<=8))&&((-8<=total_y)&&(total_y<0))){
      counter_y--;
    }else if(((0>=prev_val_y)&&(prev_val_y>=-8))&&((8>=total_y)&&(total_y>0))){
      counter_y++;
    }
  }
  

  if(((200<=prev_val_x)&&(prev_val_x<=208))&&((-208<=total_x)&&(total_x<=-200))){
    counter_x++;
  }else if(((-200>=prev_val_x)&&(prev_val_x>=-208))&&((200<=total_x)&&(total_x<=208))){
     counter_x--;
  }
  if(counter_x!=0){
    if(((0<=prev_val_x)&&(prev_val_x<=8))&&((-8<=total_x)&&(total_x<0))){
      counter_x--;
    }else if(((0>=prev_val_x)&&(prev_val_x>=-8))&&((8>=total_x)&&(total_x>0))){
      counter_x++;
    }
  }
  
  //this shows the minimum possible value y can take at the moment 
  min_y = 208*counter_y;
  min_x = 208*counter_x;

    //finding actual value of y 
  if(counter_y>0){  
    if(total_y>0){
      actual_y = min_y + total_y;
    }else if(total_y<0){
      actual_y = min_y + (208+total_y);
    }
  }else if(counter_y<0){
    if(total_y>0){
      actual_y = min_y - (208-total_y);
    }else if(total_y<0){
      actual_y = min_y + total_y;
    }
  }else{ 
    actual_y=total_y;
  }

  if(counter_x>0){  
    if(total_x>0){
      actual_x = min_x + total_x;
    }else if(total_x<0){
      actual_x = min_x + (208+total_x);
    }else{
      //for completeness
      actual_x = actual_x;
     }  
  }else if(counter_x<0){
    if(total_x>0){
      actual_x = min_x - (208-total_x);
    }else if(total_x<0){
      actual_x = min_x + total_x;
    }else{
      //for completeness
      actual_x = actual_x;
     }
  }else{ 
    actual_x=total_x;
  }

  vector_add(calc_rad(abs(actual_x)), actual_y - rover_length);
  cartesian(theta_total, r_total);

 Serial.print('\n');

Serial.print("vref = " + String(vref));
Serial.println("vb = " + String(vb));
Serial.print('\n');

Serial.println("Distance_x = " + String(total_x));
Serial.println("Distance_y = " + String(total_y));
Serial.print('\n');

Serial.print("Counter y= ");
Serial.println(counter_y);
Serial.print('\n');
Serial.print("Counter x= ");
Serial.println(counter_x);
Serial.print('\n');

Serial.print('\n');
Serial.print("Actual y = ");
Serial.println(actual_y);

Serial.print('\n');
Serial.print("Actual x = ");
Serial.println(actual_x);

Serial.print('\n');
Serial.print("prev x = ");
Serial.println(prev_val_x);

Serial.print('\n');
Serial.print("prev y = ");
Serial.println(prev_val_y);

  delay(100);

  //#endif
   
  if (Serial.available() > 0) {
    // read the incoming byte:
     char _modenew = Serial.read();
     if(_mode!=_modenew){
        _mode=_modenew;
     }
   }

  digitalWrite(DIRR, DIRRstate);
  digitalWrite(DIRL, DIRLstate);
  //*********//
  
  if(vb >= vref - 0.2){
    if(_mode == 'o'){
      if(!rover_scan_zero(_mode)){
        rover_scan_zero(_mode);
      }
      Serial.println("s0 = "+String(rover_scan_zero(_mode)));
    }else if(_mode == 'i'){
      if(!rover_scan_first(_mode)){
        rover_scan_first(_mode);
      }
      if(!reach_forward(_mode)){
        reach_forward(_mode);
      }  
      Serial.println("s1 = "+String(rover_scan_first(_mode)));
    }else{
      brake();
    }  
  }else{
    brake();}
    
  prev_val_y = total_y;
  prev_val_x = total_x;

for(int i = 0; i<6; i++){
  Serial1.println(data_command[i]);
}

for(int i = 0; i<4; i++){
  Serial1.println(data_vision[i]);
}
}
