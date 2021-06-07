#ifndef functions_h
#define functions_h

#include <INA219_WE.h>
INA219_WE ina219;

#define PIN_SS        10
#define PIN_MISO      12
#define PIN_MOSI      11
//#define PIN_SCK       13

#define PIN_MOUSECAM_RESET     8
#define PIN_MOUSECAM_CS        7

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

//#define ADNS3080_PRODUCT_ID_VAL        0x17

float open_loop, closed_loop; // Duty Cycles
float vpd,vb,vref,iL,dutyref,current_mA; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration
float ev=0,cv=0,ei=0,oc=0; //internal signals
float Ts=0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float kpv=1.5,kiv=5.78,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float kpi=0.02512,kii=39.4,kdi=0; // current pid.
float u0i,u1i,delta_ui,e0i,e1i,e2i; // Internal values for the current controller
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max=1, ui_min=0; //anti-windup limitation
float current_limit = 1.0;
boolean Boost_mode = 0;
boolean CL_mode = 0;

unsigned int loopTrigger;
unsigned int com_count=0;   // a variables to count the interrupts. Used for program debugging.

int DIRRstate = LOW;              //initializing direction states
int DIRLstate = HIGH;

int DIRL = 20;                    //defining left direction pin
int DIRR = 21;                    //defining right direction pin

int pwmr = 5;                     //pin to control right wheel speed using pwm
int pwml = 9;                     //pin to control left wheel speed using pwm
//*********//
//char _mode='g';
int c_new = 0;
int v_new = 0;
char _mode;
char c[6] = {'0','0','0','0','0','0'};
char v[2] = {'0', 'g'};

int _auto = 0;
int found = 0;

float theta = 0;
float r = 0;

float ycal_total = 0;
float xcal_total = 0;

int prev_val_y = 0;
int prev_val_x = 0;
int actual_y_prev = 0;
int min_y = 0;
int min_x = 0;

int rover_length = 73;

float cart_x = 0;
float cart_y = 0;

int search_x = 0;
int search_y = 0;
int _iter_scan = 0;
int _iter_serial = 0;

int scan_state = 0;

int counter_x = 0;
int counter_y = 0;

int total_x_distance = 0;
int total_y_distance = 0;
int total_x = 0;
int total_y = 0;
int total_x1_distance = 0;
int total_y1_distance = 0;
int total_x1 = 0;
int total_y1 = 0;

int x=0;
int y=0;

int a=0;
int b=0;

int actual_x_first = 0;
int actual_x_second = 0;
int actual_x_scan = 0;
int actual_y = 0;
int actual_x = 0;

float distance_x=0;
float distance_y=0;

volatile byte movementflag=0;
volatile int xydat[2];

int tdistance = 0;

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  
  digitalWrite(PIN_MOUSECAM_CS,HIGH);

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);

  return 0;
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(1);
  return ret;
}

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};


void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(5);
}

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
}

// This subroutine processes all of the analogue samples, creating the required values for the main loop

float saturation( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}

// This is a PID controller for the voltage

float pidv( float pid_input){
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;
 
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv*(e0v-e1v) + kiv*Ts*e_integration + kdv/Ts*(e0v-2*e1v+e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  saturation(u0v,uv_max,uv_min);
  
  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}

// This is a PID controller for the current

float pidi(float pid_input){
  float e_integration;
  e0i = pid_input;
  e_integration=e0i;
  
  //anti-windup
  if(u1i >= ui_max){
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }
  
  delta_ui = kpi*(e0i-e1i) + kii*Ts*e_integration + kdi/Ts*(e0i-2*e1i+e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i,ui_max,ui_min);
  
  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensorValue0 = analogRead(A0); //sample Vb
  vref = 1.6;
  sensorValue3 = analogRead(A3); //sample Vpd
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  sensorValue2 = vref * (1023.0 / 4.096);
  //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  if (Boost_mode == 1){
    iL = -current_mA/1000.0;
    dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    iL = current_mA/1000.0;
    dutyref = sensorValue2 * (1.0 / 1023.0);
  }
  
}

void back(){
  digitalWrite(pwmr,HIGH);
  digitalWrite(pwml,HIGH);
  DIRRstate = HIGH;
  DIRLstate = LOW;
}

void left(){
  digitalWrite(pwmr,HIGH);
  digitalWrite(pwml,HIGH);
  DIRRstate = LOW;
  DIRLstate = LOW;
}

void right(){
  digitalWrite(pwmr,HIGH);
  digitalWrite(pwml,HIGH);
  DIRRstate = HIGH;
  DIRLstate = HIGH;
}

void forward(){
  digitalWrite(pwmr,HIGH);
  digitalWrite(pwml,HIGH);
  DIRRstate = LOW;
  DIRLstate = HIGH;
}

void brake(){
  digitalWrite(pwmr,LOW);
  digitalWrite(pwml,LOW);
}

//**********************************//
//void gotocoordinate(int x, int y){
//  if(){
//    theta = atan2(y,x);
//    r = sqrt((x^2)+(y^2));
//  }
//  if(actual_x >= theta){
//    brake();
//  }else{
//    left();
//  }
//  if(actual_y >= r){
//    brake();
//  }else{
//    forward();
//  }
//}

float store_angle(float x){
    float angle_deg = ((x/850)*360);
    float angle_rad = (x/850)*2*PI;
    Serial.print("angle = ");
    Serial.println(angle_deg);
    return angle_rad;
}

float xcoordinatefinder(float old_r, float new_r, float angle){
    float xcal = (new_r - old_r)*cos(angle);
    xcal_total = xcal_total + xcal;
    Serial.print("x coordinate of rover = ");
    Serial.println(xcal_total);
    return xcal_total; 
}

float ycoordinatefinder(float old_r, float new_r, float angle){
    float ycal = (new_r - old_r)*sin(angle);
    ycal_total = ycal_total + ycal;
    Serial.print("y coordinate of rover = ");
    Serial.println(ycal_total);
    return ycal_total; 
}

bool rover_scan_one(char _mode){
  if( _mode == 's'){
    brake();
    Serial.println("Scan_one = STOP");
    scan_state = 3;
    Serial.println("_mode_one = "+String(_mode));
    delay(500);
    return true;
  }else if(_mode == 'g'){
    left();
    return false;
  }
}

bool rover_scan_zero(char _mode){
  if(_iter_scan == 0){
    actual_x_first = abs(actual_x);
  }
  if(abs(actual_x) >= 800 +actual_x_first){
    brake();
    if((_iter_scan == 1)||(_iter_scan == 2)){
      scan_state++;
      _iter_scan = 0;
    }
    Serial.println("scan_state_zero = "+String(scan_state));
    delay(500);
    return true;
  }else if(_mode == 's'){
    brake();
    if((prev_val_x == total_x)&&(_iter_scan == 1)){
//      Serial.println("angle = "+String(angle));
//      Serial.println("angle_corrected = "+String(angle_corrected));
//      Serial.println("rad = "+String(rad));
//      Serial.println("theta_total = "+String(theta_total));
//      Serial.println("r_total = "+String(r_total));
//      Serial.println("cart_x = "+String(cart_x));
//      Serial.println("cart_y = "+String(cart_y));
      _iter_scan = 2;
    }
    Serial.println("Scan_zero = STOP");
    return false;
  }else{
    if(_mode == 'g'){
      left();
      _iter_scan = 1;
      return false;
    }
  }
}

void rover_manual(char _mode){
  if (_mode == 'w') {
      forward();
  }  
  //rotating clockwise
  if (_mode == 'd') {
    right();
  }
  if (_mode == 's') {
    back();
  }
  //rotating anticlockwise
  if (_mode == 'a') {
    left();
  } 
  if(_mode == 'x'){
    brake();
  }
}

bool reach_forward(char _mode){
    if(_mode == 'g'){
      vref = 3;
      forward();
      return false;
    }else if(_mode == 's'){
        brake();
        scan_state = 5;
        return true;
    }else if(_mode == 'r'){
        right();
        return false;
    }else if(_mode == 'l'){
        left();
        return false;
    }
}

//{"0d14.1+0003-0012"}
//String data_command[] = {send_to, gear, x, y, total, _speed};
//String data_vision[] = {"v", s0 bool, s1 bool, theta, r};
String data_command[] = {"c", "RR", "+0003.4", "-0015.2", "00003422", "10.2"};
String data_vision[] = {"v", String(rover_scan_zero(_mode)), "0057.4", "0132.0"};

#endif 
