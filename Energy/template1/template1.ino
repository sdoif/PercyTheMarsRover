#include <MatrixMath.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 10;
unsigned int rest_timer;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_measure, current_ref = 0, error_amps; // Current Control
float pwm_out;
float V_Bat;
float Vbat1;
float Vbat2;
float Vbat3;
float Vbat4;
boolean input_switch;
int state_num=0,next_state;
String dataString;
String ocvString;
int count = 1;
int soc_count = 1;
int soc = 100;
int ocv_count;

using namespace BLA;

BLA::Matrix<2,2> identityMat = { 1, 0,
                                   0, 1};
  BLA::Matrix<2,1> cond_bat1 = {soc,
                                       0};
  BLA::Matrix<2,2> trans_mat = {1, -400/1800,
                                0,    1};
  BLA::Matrix<2,2> obs_mat = {1, 0,
                              0, 1};
  BLA::Matrix<2,2> obs_noise_mat = {0.25, 0,
                                    0, 0.25};
  BLA::Matrix<2,2> est_var_mat = {5^2, 0,
                                  0, 5^2};
  BLA::Matrix<2,2> trans_mat_t = ~trans_mat;
  BLA::Matrix<2,2> obs_mat_t = ~obs_mat;
  BLA::Matrix<2,1> new_state_est;
  BLA::Matrix<2,2> new_est_var1;
  BLA::Matrix<2,2> new_est_var;
  BLA::Matrix<2,1> ocv_measure_mat;
  BLA::Matrix<2,1> y;
  BLA::Matrix<2,1> z;
  BLA::Matrix<2,1> hx;
  BLA::Matrix<2,2> res_var_mat1;
  BLA::Matrix<2,2> res_var_mat2;
  BLA::Matrix<2,2> res_var_mat;
  BLA::Matrix<2,2> inv_res_var_mat;
  BLA::Matrix<2,2> kal_gain;
  BLA::Matrix<2,2> kal_gain1;
  BLA::Matrix<2,1> zkal_gain;
  BLA::Matrix<2,2> iden_kal_gain;
  BLA::Matrix<2,2> new_best_est_var_mat;


void setup() {
  //Some General Setup Stuff

  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications


  //Check for the SD Card
  Serial.println("\nInitializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("* is a card inserted?");
    while (true) {} //It will stick here FOREVER if no SD is in on boot
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (SD.exists("BatCycle.csv")) { // Wipe the datalog when starting
    SD.remove("BatCycle.csv");
  }

  
  noInterrupts(); //disable all interrupts
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  //SMPS Pins
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);

  // TimerA0 initialization for 1kHz control-loop interrupt.ç
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, 120); //just a default state to start with

  
}  

void loop() {
  if (loop_trigger == 1){ // FAST LOOP (1kHZ)
      state_num = next_state; //state transition
      V_Bat = analogRead(A0)*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
      if ((V_Bat > 3700 || V_Bat < 2400)) { //Checking for Error states (just battery voltage for now)
          state_num = 5; //go directly to jail
          next_state = 5; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          current_ref = 0; // no current
      }
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  }

  if (count == 1){
    switch(ocv_count){
      case 1:{
        digitalWrite(5, true);
        for (int i = 1; i<1000; i++){
          
        }
        Vbat1 = analogRead(A7)*4.096/1.03;
        digitalWrite(5, false);
        ocv_count++;
        break;
      }
      case 2: {
        digitalWrite(4, true);
        for (int i = 1; i<1000; i++){
          
        }
        Vbat2 = analogRead(A7)*4.096/1.03;
        digitalWrite(4, false);
        ocv_count++;
        break;
      }
      case 3: {
        digitalWrite(9, true);
        for (int i = 1; i<1000; i++){
          
        }
        Vbat3 = analogRead(A7)*4.096/1.03;
        digitalWrite(9, false);
        ocv_count++;
        break;
      }
      case 4: {
        digitalWrite(3, true);
        for (int i = 1; i<1000; i++){
          
        }
        Vbat4 = analogRead(A7)*4.096/1.03;
        digitalWrite(3, false);
        ocv_count = 1; 
        break;
      }}
      if (soc_count == 1) {
        cond_bat1(2,1) = ((current_measure*Vbat1)/(Vbat2 + Vbat3 + Vbat4));

        Multiply(trans_mat, cond_bat1, new_state_est);
        Multiply(trans_mat, est_var_mat, new_est_var1);
        Multiply(new_est_var1, trans_mat_t, new_est_var);

        ocv_measure_mat(1,1) = Vbat1;
        ocv_measure_mat(2,1) = ((3*current_measure*Vbat1)/(Vbat2 + Vbat3 + Vbat4));

        Multiply(obs_mat, ocv_measure_mat, y);
        Multiply(obs_mat, new_state_est, hx);

        z = y - hx;

        Multiply(obs_mat, new_est_var, res_var_mat1);
        Multiply(res_var_mat1, obs_mat_t, res_var_mat2);
        Add(res_var_mat2, obs_noise_mat, res_var_mat);
        
        inv_res_var_mat = res_var_mat.Inverse();
        
        Multiply(new_est_var, obs_mat_t, kal_gain1);
        Multiply(kal_gain1, inv_res_var_mat, kal_gain);
        Multiply(kal_gain, z, zkal_gain);
        Add(zkal_gain, new_state_est, cond_bat1);
        Subtract(identityMat, kal_gain, iden_kal_gain);
        Multiply(new_est_var, iden_kal_gain, new_best_est_var_mat);
        new_est_var = new_best_est_var_mat;
        
         
      }
      
    }
  

  
    
  
  if (int_count == 1000) { // SLOW LOOP (1Hz)
    input_switch = digitalRead(2); //get the OL/CL switch status
    switch (state_num) { // STATE MACHINE (see diagram)
      case 0:{ // Start state (no current, no LEDs)
        current_ref = 0;
        if (input_switch == 1) { // if switch, move to charge
          next_state = 1;
          digitalWrite(8,true);
        } else { // otherwise stay put
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 1:{ // Charge state (250mA and a green LED)
        current_ref = 250;
        if (V_Bat < 3600) { // if not charged, stay put
          next_state = 1;
          digitalWrite(8,true);          
        } else { // otherwise go to charge rest
          next_state = 2;
          digitalWrite(8,false);
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 2:{ // Charge Rest, green LED is off and no current
        current_ref = 0;
        if (rest_timer < 30) { // Stay here if timer < 30
          next_state = 2;
          digitalWrite(8,false);
          rest_timer++;
        } else { // Or move to discharge (and reset the timer)
          next_state = 3;
          digitalWrite(8,false);
          rest_timer = 0;
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;        
      }
      case 3:{ //Discharge state (-250mA and no LEDs)
         current_ref = -250;
         if (V_Bat > 2500) { // While not at minimum volts, stay here
           next_state = 3;
           digitalWrite(8,false);
         } else { // If we reach full discharged, move to rest
           next_state = 4;
           digitalWrite(8,false);
         }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 4:{ // Discharge rest, no LEDs no current
        current_ref = 0;
        if (rest_timer < 30) { // Rest here for 30s like before
          next_state = 4;
          digitalWrite(8,false);
          rest_timer++;
        } else { // When thats done, move back to charging (and light the green LED)
          next_state = 1;
          digitalWrite(8,true);
          rest_timer = 0;
        }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 5: { // ERROR state RED led and no current
        current_ref = 0;
        next_state = 5; // Always stay here
        digitalWrite(7,true);
        digitalWrite(8,false);
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(7,false);
        }
        break;
      }

      default :{ // Should not end up here ....
        Serial.println("Boop");
        current_ref = 0;
        next_state = 5; // So if we are here, we go to error
        digitalWrite(7,true);
      }
      
    }
    
    dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(cond_bat1(2,1)); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
  }
}

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}

float saturation( float sat_input, float uplim, float lowlim) { // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

float pidi(float pid_input) { // discrete PID function
  float e_integration;
  e0i = pid_input;
  e_integration = e0i;

  //anti-windup
  if (u1i >= ui_max) {
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }

  delta_ui = kpi * (e0i - e1i) + kii * Ts * e_integration + kdi / Ts * (e0i - 2 * e1i + e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i, ui_max, ui_min);

  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}
