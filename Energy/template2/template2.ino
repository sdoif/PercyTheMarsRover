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
float Bat_volt;
float soc_bat1;
float soc_bat2;
float soc_bat3;
float soc_bat4;
boolean input_switch;
boolean voltage_check = 1;
int state_num=0,next_state;
int voltage_bat_count;
int bat_count;
int bat_number;
int count;
String dataString;
int soc_count;
float Vbat1;
float Vbat2;
float Vbat3;
float Vbat4;
float sensorpv_voltage_pos;
float sensorpv_current_pos;
float sensorpv_voltage_neg;
float sensorpv_current_neg;
float voltage_current = 0;
float voltage_prev = 0;
float current_current = 0;
float current_prev = 0;
float power_current = 0;
float power_prev = 0;


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
      bat_count = bat_number;
      V_Bat = analogRead(A0)*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
      Bat_volt = analogRead(A1)*4.096;
      sensorpv_voltage_pos = analogRead(A2)*4.096;
      sensorpv_voltage_neg = analogRead(A3)*4.096;
      sensorpv_current_pos = analogRead(A6)*4.096;
      sensorpv_current_neg = analogRead(A7)*4.096;
      if ((V_Bat > 3700 || V_Bat < 2400)) { //Checking for Error states (just battery voltage for now)
          state_num = 5; //go directly to jail
          next_state = 5; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          current_ref = 0; // no current
      }
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      //pwm_out = mppt(sensorpv_voltage_pos, sensorpv_voltage_neg, sensorpv_current_pos, sensorpv_current_neg);
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
      voltage_bat_count++;
      soc_count++;
  }

  if(count = 1){
    input_switch = digitalRead(2);
    switch(bat_count){
      case 0:{
        if(input_switch == 1){
          bat_number = 1;
        } else {
          bat_number = 0;
          digitalWrite(7, true);
        } 
        break;
      }
      case 1:{
        digitalWrite(5, true);
        if(voltage_bat_count < 1000){
          bat_number = 1;
        } else {
          Vbat1 = Bat_volt;
          digitalWrite(5, false);
          voltage_bat_count = 0;
          bat_number = 2;
        } if(input_switch == 0){
          digitalWrite(5, false);
          bat_number = 0;
        }
        Serial.print("Reached Break");
        break;
      }
      case 2:{
        digitalWrite(4, true);
        if(voltage_bat_count < 1000){
          bat_number = 2;
        } else {
          Vbat2 = Bat_volt;
          digitalWrite(4, false);
          voltage_bat_count = 0;
          bat_number = 3;
        } if(input_switch == 0){
          digitalWrite(4, false);
          bat_number = 0;
        }
        break;
      }
      case 3:{
        digitalWrite(3, true);
        if(voltage_bat_count < 1000){
          bat_number = 3;
        } else {
          Vbat3 = Bat_volt;
          digitalWrite(3, false);
          voltage_bat_count = 0;
          bat_number = 4;
        } if(input_switch == 0){
          digitalWrite(3, false);
          bat_number = 0;
        }
        break;
      }
      case 4:{
        digitalWrite(9, true);
        if(voltage_bat_count < 1000){
          bat_number = 4;
        } else {
          Vbat4 = Bat_volt;
          digitalWrite(9, false);
          voltage_bat_count = 0;
          bat_number = 1;
        }if(input_switch == 0){
          digitalWrite(9, false);
          bat_number = 0;
        }
      }
    }
  }

  if(voltage_check == 1){
    if(Vbat1 < 3700 ){
      soc_bat1 = 2.1732e-12*pow(Vbat1,9) - 1.0437e-9*pow(Vbat1,8) + 2.1268e-7*pow(Vbat1,7) - 2.393e-5*pow(Vbat1,6) + 0.0016*pow(Vbat1,5) - 0.0676*pow(Vbat1,4) + 1.7050*pow(Vbat1,3) - 24.6247*pow(Vbat1,2) + 184.8344*Vbat1 + 2.7434e3;
    }else if(Vbat1 > 3700){
      next_state = 5;
    }
    if(Vbat2 < 3700){
      soc_bat2 = 1;
    } else if(Vbat2 > 3700){
      next_state = 5;
    }if(Vbat3 < 3700){
      soc_bat3 = 2.1149e-12*pow(Vbat3,9) - 1.0141e-9*pow(Vbat3,8) + 2.0634e-7*pow(Vbat3,7) - 2.3186e-5*pow(Vbat3,6) + 0.0016*pow(Vbat3,5) - 0.0654*pow(Vbat3,4) + 1.6475*pow(Vbat3,3) - 23.7743*pow(Vbat3,2) + 178.4581*Vbat3 + 2.7643e3;
    } else if(Vbat3 > 3700){
      next_state = 5;
    }if(Vbat4 < 3700){
      soc_bat4 = 2.8827e-12*pow(Vbat4,9) - 1.3775e-9*pow(Vbat4,8) + 2.7935e-7*pow(Vbat4,7) - 3.13e-5*pow(Vbat4,6) + 0.0021*pow(Vbat4,5) - 0.0879*pow(Vbat4,4) + 2.2175*pow(Vbat4,3) - 32.0205*pow(Vbat4,2) + 238.7472*Vbat4 + 2.5881e3;
    } else if(Vbat4 > 3700){
      next_state = 5;
    }
    voltage_check = 0;
  }
  
  if(soc_count == 1000)
  {
    if(soc_bat1 < 100){
    soc_bat1 = soc_bat1 + (100*current_measure/(3*500*3600));
    } else {
      voltage_check = 1;
      soc_bat1 = 2.1732e-12*pow(Vbat1,9) - 1.0437e-9*pow(Vbat1,8) + 2.1268e-7*pow(Vbat1,7) - 2.393e-5*pow(Vbat1,6) + 0.0016*pow(Vbat1,5) - 0.0676*pow(Vbat1,4) + 1.7050*pow(Vbat1,3) - 24.6247*pow(Vbat1,2) + 184.8344*Vbat1 + 2.7434e3;
    }  if(soc_bat2 < 100){
    soc_bat2 = soc_bat2 + (100*current_measure/(3*500*3600));
    } else {
      voltage_check = 1;
      soc_bat2 = 1;
    } if(soc_bat3 < 100){
    soc_bat3 = soc_bat3 + (100*current_measure/(3*500*3600));
    } else {
      voltage_check = 1;
      soc_bat3 = 2.1149e-12*pow(Vbat3,9) - 1.0141e-9*pow(Vbat3,8) + 2.0634e-7*pow(Vbat3,7) - 2.3186e-5*pow(Vbat3,6) + 0.0016*pow(Vbat3,5) - 0.0654*pow(Vbat3,4) + 1.6475*pow(Vbat3,3) - 23.7743*pow(Vbat3,2) + 178.4581*Vbat3 + 2.7643e3;
    } if (soc_bat4 < 100){
    soc_bat4 = soc_bat4 + (100*current_measure/(3*500*3600));
    } else {
      voltage_check = 1;
      soc_bat4 = 2.8827e-12*pow(Vbat4,9) - 1.3775e-9*pow(Vbat4,8) + 2.7935e-7*pow(Vbat4,7) - 3.13e-5*pow(Vbat4,6) + 0.0021*pow(Vbat4,5) - 0.0879*pow(Vbat4,4) + 2.2175*pow(Vbat4,3) - 32.0205*pow(Vbat4,2) + 238.7472*Vbat4 + 2.5881e3;
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
        if (Vbat1 < 3600 && soc_bat1 <100 && Vbat2 < 3600 && soc_bat2 <100 && Vbat3 < 3600 && soc_bat3 <100 && Vbat4 < 3600 && soc_bat4 <100) { // if not charged, stay put
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
          voltage_check = 1;
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
         if (Vbat1 > 2750 && soc_bat1 > 0 && Vbat2 > 2900 && soc_bat2 > 0 && Vbat3 > 2770 && soc_bat3 > 0 && Vbat4 > 2600 && soc_bat4 > 0) { // While not at minimum volts, stay here
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
          voltage_check = 1;
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
    
    dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(Vbat1) + "," + String(soc_bat1) + "," + String(Vbat2) + "," + String(soc_bat2) + "," + String(Vbat3) + "," + String(soc_bat3) + "," + String(Vbat4) + "," + String(soc_bat4); //build a datastring for the CSV file
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

float mppt (float voltage_pos, float voltage_neg, float current_pos, float current_neg){

  voltage_current = voltage_pos - voltage_neg;
  current_current = (current_pos - current_neg)/10000;

  power_current = voltage_current*current_current;

  if(power_current > power_prev){

    if(voltage_current > voltage_prev){
      pwm_out = pwm_out - 0.05;
    } else {
      pwm_out = pwm_out + 0.05;
    }
  } else {
    if(voltage_current > voltage_prev){
      pwm_out = pwm_out + 0.05;
    } else {
      pwm_out = pwm_out - 0.05;
    }
  }
    if(pwm_out < 0.01){
      pwm_out = 0.01;
    }
    if(pwm_out > 0.99){
      pwm_out = 0.99;
    }
  return pwm_out;
  
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
