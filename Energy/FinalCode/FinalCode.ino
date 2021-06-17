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
int voltage_bat_count = 0;
int bat_count = 0;
int bat_number = 0;
int count = 1;
String dataString;
int soc_count;
float Vbat1 = 3;
float Vbat2 = 3;
float Vbat3 = 3;
float Vbat4 = 3;
float sensorpv_voltage;
float sensorpv_current_pos;
float sensorpv_current_neg;
float soh_bat1;
float soh_bat2;
float soh_bat3;
float charge_bat1;
float charge_bat2;
float charge_bat3;
float voltage_current = 0;
float voltage_prev = 0;
float current_current = 0;
float current_prev = 0;
float power_current = 0;
float power_prev = 0;
int initialize = 1;
int balance_timer;


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
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

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
      sensorpv_voltage = analogRead(A2)*4.096/0.3708;
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
      if(state_num == 1){
        pwm_out = mppt(sensorpv_voltage, sensorpv_current_pos, sensorpv_current_neg);
      }else{
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      }
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
      voltage_bat_count++;
      soc_count++;
  }

    input_switch = digitalRead(2);
    switch(bat_count){
      case 0:{
        if(input_switch == 1){
          bat_number = 1;
          digitalWrite(7, false);
        } else {
          bat_number = 0;
          digitalWrite(7, true);
        } 
      }
      case 1:{
        digitalWrite(5, true);
        if(voltage_bat_count < 1000){
          bat_number = 1;
        } else {
          Vbat1 = Bat_volt/1000;
          digitalWrite(5, false);
          voltage_bat_count = 0;
          bat_number = 2;
        } if(input_switch == 0){
          digitalWrite(5, false);
          bat_number = 0;
        }
        break;
      }
      case 2:{
        digitalWrite(4, true);
        if(voltage_bat_count < 1000){
          bat_number = 2;
        } else {
          Vbat2 = Bat_volt/1000;
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
          Vbat3 = Bat_volt/1000;
          digitalWrite(3, false);
          voltage_bat_count = 0;
          bat_number = 1;
        } if(input_switch == 0){
          digitalWrite(3, false);
          bat_number = 0;
        }
        break;
      }

    }
  

  if(voltage_check == 10000){
    if(Vbat1 < 3.7 ){
      if(Vbat1 < 3.28078){
        soc_bat1 = 12.915*Vbat1 - 34.9624;
      } else if(Vbat1 < 3.31259){
        soc_bat1 = 305.20716*Vbat1 - 993.9086;
      } else if(Vbat1 < 3.35633){
        soc_bat1 = 248.251*Vbat1 - 805.236;
      } else if(Vbat1 < 3.37224){
        soc_bat1 = 1303.3878*Vbat1 - 4346.6236;
      } else if(Vbat1 < 3.4){
        soc_bat1 = 602.0317*Vbat1 - 1981.48248;
      } else if(Vbat1 < 3.42791){
        soc_bat1 = 646.5496*Vbat1 - 2132.843;
      } else if(Vbat1 < 3.46371){
        soc_bat1 = 243.8156*Vbat1 - 752.307;
      } else if(Vbat1 < 3.6){
        soc_bat1 = 57.409*Vbat1 - 106.649;
      }
    }else if(Vbat1 > 3.7){
     next_state = 5; 
    }
    if(Vbat2 < 3.7){
      if(Vbat2 < 3.27282){
        soc_bat2 = 12.3879*Vbat2 + 2.79164;
      } else if(Vbat2 < 3.33247){
        soc_bat2 = 241.851*Vbat2 - 786.451;
      } else if(Vbat2 < 3.36429){
        soc_bat2 = 195.408*Vbat2 - 631.68;
      } else if(Vbat2 < 3.37224){
        soc_bat2 = 1733.597*Vbat2 - 5806.595;
      } else if(Vbat2 < 3.4){
        soc_bat2 = 761.027*Vbat2 - 2526.855;
      } else if(Vbat2 < 3.44382){
        soc_bat2 = 665.903*Vbat2 - 2203.4336;
      } else if(Vbat2 < 3.47564){
        soc_bat2 = 183.8686*Vbat2 - 543.39396;
      } else if(Vbat2 < 3.60687){
        soc_bat2 = 33.0176*Vbat2 - 19.09;
      }
    } else if(Vbat2 > 3.7){
      next_state = 5;
    }if(Vbat3 < 3.7){
      if(Vbat3 < 3.28873){
        soc_bat3 = 15.951*Vbat3 - 44.2822;
      } else if(Vbat3 < 3.30464){
        soc_bat3 = 504.219*Vbat3 - 1650.0634;
      } else if(Vbat3 < 3.35236){
        soc_bat3 = 148.950*Vbat3 - 476.0273;
      } else if(Vbat3 < 3.36429){
        soc_bat3 = 599.044*Vbat3 - 1984.906;
      } else if(Vbat3 < 3.37622){
        soc_bat3 = 1109.564*Vbat3 - 3702.442;
      } else if(Vbat3 < 3.43189){
        soc_bat3 = 726.524*Vbat3 - 2409.2144;
      } else if(Vbat3 < 3.48545){
        soc_bat3 = 201.8895*Vbat3 - 608.7264;
      } else if(Vbat3 < 3.59494){
        soc_bat3 = 45.8946*Vbat3 - 65.01413;
      }
    } else if(Vbat3 > 3.7){
      next_state = 5;

    voltage_check = 0;
  }
  }
  
  if(soc_count == 1000)
  {
    if(soc_bat1 < 100){
    soc_bat1 = soc_bat1 + (current_measure/(5*3600));
    charge_bat1 = charge_bat1 + (current_measure/(500*3600));
    } else {
      voltage_check = 10000;
      if(Vbat1 < 3.28078){
        soc_bat1 = 12.915*Vbat1 - 34.9624;
      } else if(Vbat1 < 3.31259){
        soc_bat1 = 305.20716*Vbat1 - 993.9086;
      } else if(Vbat1 < 3.35633){
        soc_bat1 = 248.251*Vbat1 - 805.236;
      } else if(Vbat1 < 3.37224){
        soc_bat1 = 1303.3878*Vbat1 - 4346.6236;
      } else if(Vbat1 < 3.4){
        soc_bat1 = 602.0317*Vbat1 - 1981.48248;
      } else if(Vbat1 < 3.42791){
        soc_bat1 = 646.5496*Vbat1 - 2132.843;
      } else if(Vbat1 < 3.46371){
        soc_bat1 = 243.8156*Vbat1 - 752.307;
      } else if(Vbat1 < 3.6){
        soc_bat1 = 57.409*Vbat1 - 106.649;
      }
    }  if(soc_bat2 < 100){
    soc_bat2 = soc_bat2 + (current_measure/(5*3600));
    charge_bat2 = charge_bat2 + (current_measure/(500*3600));
    } else {
      voltage_check = 10000;
      if(Vbat2 < 3.27282){
        soc_bat2 = 12.3879*Vbat2 + 2.79164;
      } else if(Vbat2 < 3.33247){
        soc_bat2 = 241.851*Vbat2 - 786.451;
      } else if(Vbat2 < 3.36429){
        soc_bat2 = 195.408*Vbat2 - 631.68;
      } else if(Vbat2 < 3.37224){
        soc_bat2 = 1733.597*Vbat2 - 5806.595;
      } else if(Vbat2 < 3.4){
        soc_bat2 = 761.027*Vbat2 - 2526.855;
      } else if(Vbat2 < 3.44382){
        soc_bat2 = 665.903*Vbat2 - 2203.4336;
      } else if(Vbat2 < 3.47564){
        soc_bat2 = 183.8686*Vbat2 - 543.39396;
      } else if(Vbat2 < 3.60687){
        soc_bat2 = 33.0176*Vbat2 - 19.09;
      }
    } if(soc_bat3 < 100){
    soc_bat3 = soc_bat3 + (current_measure/(5*3600));
    charge_bat3 = charge_bat3 + (current_measure/(500*3600));
    } else {
      voltage_check = 10000;
      if(Vbat3 < 3.28873){
        soc_bat3 = 15.951*Vbat3 - 44.2822;
      } else if(Vbat3 < 3.30464){
        soc_bat3 = 504.219*Vbat3 - 1650.0634;
      } else if(Vbat3 < 3.35236){
        soc_bat3 = 148.950*Vbat3 - 476.0273;
      } else if(Vbat3 < 3.36429){
        soc_bat3 = 599.044*Vbat3 - 1984.906;
      } else if(Vbat3 < 3.37622){
        soc_bat3 = 1109.564*Vbat3 - 3702.442;
      } else if(Vbat3 < 3.43189){
        soc_bat3 = 726.524*Vbat3 - 2409.2144;
      } else if(Vbat3 < 3.48545){
        soc_bat3 = 201.8895*Vbat3 - 608.7264;
      } else if(Vbat3 < 3.59494){
        soc_bat3 = 45.8946*Vbat3 - 65.01413;
      }

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
        if (Vbat1 < 3.6 && soc_bat1 <100 && Vbat2 < 3.6 && soc_bat2 <100 && Vbat3 < 3.6 && soc_bat3 <100) { // if not charged, stay put
          next_state = 1;
          digitalWrite(8,true);    
          if(Vbat1 - 50 > (Vbat2 || Vbat3))
          {
            next_state = 6;
          }else if(Vbat2 - 50 > (Vbat1 || Vbat3))
          {
            next_state = 6;
          } else if(Vbat3 - 50 > (Vbat2 || Vbat1))
          {
            next_state = 6;
          } 
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
          voltage_check = 10000;
          soh_bat1 = charge_bat1/(490*3600);
          soh_bat2 = charge_bat2/(498*3600);
          soh_bat3 = charge_bat3/(496*3600);
        } else { // Or move to discharge (and reset the timer)
          next_state = 3;
          charge_bat1 = 0;
          charge_bat2 = 0;
          charge_bat3 = 0;
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
         if (Vbat1 > 2.7 && soc_bat1 > 0 && Vbat2 > 2.8 && soc_bat2 > 0 && Vbat3 > 2.770 && soc_bat3 > 0) { // While not at minimum volts, stay here
           next_state = 3;
           digitalWrite(8,false);
          
          if(Vbat1 - 50 > (Vbat2 || Vbat3))
          {
            next_state = 7;
          }else if(Vbat2 - 50 > (Vbat1 || Vbat3))
          {
            next_state = 7;
          } else if(Vbat3 - 50 > (Vbat2 || Vbat1))
          {
            next_state = 7;
          } 

         
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
          voltage_check = 10000;
          soh_bat1 = -charge_bat1/(490*3600);
          soh_bat2 = -charge_bat2/(498*3600);
          soh_bat3 = -charge_bat3/(496*3600);
        } else { // When thats done, move back to charging (and light the green LED)
          next_state = 1;
          charge_bat1 = 0;
          charge_bat2 = 0;
          charge_bat3 = 0;
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

      case 6: { //Balance State Charge
        if(balance_timer < 30){
          next_state = 6;
          digitalWrite(8, false);
          balance_timer++;
        } else{
          next_state = 1;
          digitalWrite(8, true);
          balance_timer = 0;
        }
        break;
      }

      case 7: { //Balance State Discharge
         if(balance_timer < 30){
          next_state = 7;
          digitalWrite(8, false);
          balance_timer++;
        } else{
          next_state = 3;
          digitalWrite(8, true);
          balance_timer = 0;
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
    
    dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(Vbat1) + "," + String(soc_bat1) + "," + String(Vbat2) + "," + String(soc_bat2) + "," + String(Vbat3) + "," + String(soc_bat3); //build a datastring for the CSV file
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

float mppt (float voltage, float current_pos, float current_neg){

  voltage_current = voltage;
  current_current = (current_pos - current_neg)/1000;

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
