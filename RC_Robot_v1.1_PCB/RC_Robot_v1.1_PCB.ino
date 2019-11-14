/*
  RC Platform
  Adapted by Chris Rybitski 2018

  Update Log:
  9/22/19 adapted code to run on RC Platform v1.1 hardware
          added support for LED, button, and DIP switches
  10/8/19 added code in setup to change sabertooth 2x60 baud from default 9600 to 115200

   Connections:
   Sabertooth 2x60 DIP Settings:
   1   2   3   4   5   6
  OFF OFF  ON  ON  ON  ON

   Sabertooth > Arduino Mega
           S1 > TX3 (Pin 14)
     Connect a 10K resistor from 5V to RX2
     This pulls the serial port HIGH so that it doesn't read spurious data should the receiver fail.

  SyRen  > Arduino
      S1 > TX1 (Pin 18)

   IBUS  > Arduino
    IBUS > RX2 (pin 17)


  Brake relay > pin 9
  Estop signal > pin 8

  Configuration Jumpers
                       10K
     5v----o__o----5--/\/\/---GND
                       10K
     5v----o__o----6--/\/\/---GND
                       10K
     5v----o__o----7--/\/\/---GND
                       10K
     5v----o__o----8--/\/\/---GND

*/
#include "FlySkyIBus.h"
#include <Sabertooth.h>

#include <iBUStelemetry.h>    //ibus sensor library
iBUStelemetry telemetry(12);  //connected to pin 12

Sabertooth ST(128, Serial3); // The Sabertooth is on address 128. We'll name its object ST.
Sabertooth SyRen(129, Serial1); //SyRen for linear actuarot

const int vSense = A0;  //voltage divider
const int brake = 3;    //brake is on pin 9
const int estop = 2;    //input from wireless estop
const int set1 = 5;     //DIP switch settings
const int set2 = 6;     //DIP switch settings
const int set3 = 7;     //DIP switch settings
const int set4 = 8;     //DIP switch settings
const int button = 47;  //button on PCB
const int LedR = 9;     //RGB LED
const int LedG = 10;    //RGB LED
const int LedB = 11;    //RGB LED
const int D44 = 44;     //Additional Output Pin
const int D45 = 45;     //Additional Output Pin
const int D46 = 46;     //Additional Output Pin

int steering;
int right;
int left;
int knob1;
int knob2;
int lift;
int SwC;
int SwA;
int SwB;
int SwD;

//values for ramping configuration
int rampMin;
int rampMax;

int heartBeat = 0;                       //counter to detect if IBUS is transmitting data
const int heartBeatTimeout = 100;              //counts required to trigger loss of IBUS routine

//flags
bool failSafeFlag = true;              //tracks failsafe status true = failsafe is triggered
bool estopFlag = false;                //tracks estop status true = estop is activated
bool ledEnable = false;                //set by a dip switch to control LED output

int counter = 0;                       //used for auto brake
const int autoBrakeThreshold = 700;    //counts requred to activate auto brake when ramping is set to low 700 is the shortest time before the motor stops
bool autoBrake = true;                 //stores the state of the brake

bool mixedMode = false;                //mixed mode variable for 1 or 2 joystick control

//RX Variables
char rightPower;    //stroes mapped value for sabertooth motor
char leftPower;     //stroes mapped value for sabertooth motor
char maxSpeed;      //stroes mapped value for max speed of motor
char ramp;          //stroes mapped value rampping curve on sabertooth
char oldRamp = 0;   //stroes mapped value for sabertooth motor
char liftSpeed;

//data
float voltage = 0;    //voltage x10 read from ADC
long previousMillis; //used to reduce frequency of ADC reads
long interval = 1000; //change this number to increase or decrease frequency of ADC Readings


void setup()
{
  Serial.begin(115200);     //for debugging

  IBus.begin(Serial2);      //for radio

  /*
    By default the Sabertooth 2x60 is set to 9600 baud. For the first run this will need to be changed.
    Once this code is run once (with the ST2x60 connected and powered up) the new baud rate will be saved in EEPROM.
    The below lines are only needed once, but it is just easier to leave them in...
  */
  Serial3.begin(9600);
  ST.setBaudRate(115200);     //change baud rate to 115200 just incase it isn't already.
  delay(100);                 //give the Sabertooth a second to breath

  Serial3.begin(115200);      // set baud rate for sabertooth



  Serial1.begin(9600);
  Sabertooth::autobaud(Serial1); // Autobaud is for the whole serial line

  // ST.setBaudRate(9600);
  ST.setTimeout(1);         //turn off motors if no serial data for more than 100ms
  ST.setRamping(0);         //turn ramping off by default

  // This setting does not persist between power cycles.
  // On a Sabertooth 2x25, the value is (Desired Volts - 6) X 5.
  //  ST.setMinVoltage(89);     //23.8v is fully discharged value of 79 for 24v battery bank
  //-----------------was not working at last test--------------------------------

  telemetry.begin(115200);    //setup software serial for ibus sensor
  telemetry.addSensor(0);     //sensor type 0=voltage 1=temp


  pinMode(brake, OUTPUT);
  pinMode(estop, INPUT);
  pinMode(set1, INPUT);
  pinMode(set2, INPUT);
  pinMode(set3, INPUT);
  pinMode(set4, INPUT);
  pinMode(button, INPUT);
  pinMode(LedR, OUTPUT);
  pinMode(LedG, OUTPUT);
  pinMode(LedB, OUTPUT);
  pinMode(D44, OUTPUT);
  pinMode(D45, OUTPUT);
  pinMode(D46, OUTPUT);

  //Turn off LED--------------
  digitalWrite(LedR, HIGH);
  digitalWrite(LedG, HIGH);
  digitalWrite(LedB, HIGH);


  /* Configuration DIP Switches

      ---Ramping Speed---
      Fast:   Switch#1 OFF and Switch #2 OFF
      Medium: Switch#1 ON and Switch #2 OFF
      Slow:   Switch #1 OFF and Switch #2 ON

      ---Drive Mode---
      Mixed:    Switch #3 ON
      Standard: Switch #3 OFF

      ---LED---
      Enable:   Switch #4 ON
      Disable:  Switch #4 OFF

  */



  //Switch #1 and Switch #2
  if (digitalRead(set1) == HIGH && digitalRead(set2) == LOW) {
    rampMin = 21;
    rampMax = 80;        //21-80 = intermediate ramp
  }
  else if (digitalRead(set1) == LOW && digitalRead(set2) == HIGH) {
    rampMin = 11;     //11-20 = slow ramp
    rampMax = 20;
  }
  else if (digitalRead(set1) == LOW && digitalRead(set2) == LOW) {
    rampMin = 1;
    rampMax = 10;        //1-10 = fast ramp
  }

  //Switch #3
  if (digitalRead(set3) == HIGH) {
    mixedMode = true;
  }

  //Switch #4
  if (digitalRead(set4) == HIGH) {
    ledEnable = true;
  }
}

void loop()
{
  IBus.loop();                          //capture IBUS from receiver
  RX();                                 //extract IBUS data

  /*
    Voltage divider on A0
    Voltage  ADC
    24.00    849
    24.60    870
    25.00    884
    25.20    891
    26.00    919
    27.00    955
    28.85    1020

    y = ADC value and x = voltage
    x = (y/m)-(b/m)
    voltage = (ADC/35)-(0.25714)
    m= 35.55  b = 11.77
    Modified the .cpp file of the ibustelemetry library to allow decimals to be sent.
    Removed " *100 " after value. THis allowed for the correct data to be displayed on the controller.

    void iBUStelemetry::setSensorMeasurement(byte adr, float value){
    switch(sensorType[adr]){
    case 0:
      sensorValue[adr] = (int)value*100.0;
  */
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    voltage = (analogRead(vSense) / 35.55) - 0.3310;
    Serial.print(voltage);
    Serial.print("     ADC:");
    Serial.println(analogRead(vSense));
    telemetry.setSensorMeasurement(1, voltage * 100);  //(address, value) multiply by 100 due to library modificaiton
  }

  if (heartBeat > heartBeatTimeout) {   //IBUS has been missing for too long cut the motors
    failsafe();
  }

  else {                               //only run drive method if IBUS is alive

    if (digitalRead(estop) == HIGH && IBus.readChannel(9) >= 995) {   //Detect Failsafe. Radio is setup to send 990 as falisafe.

      if (ledEnable == true) {            //only turn on LED if enabled by dip switch
        digitalWrite(LedR, HIGH);         //Turn LED Green
        digitalWrite(LedG, LOW);
        digitalWrite(LedB, HIGH);
      }
      if (estopFlag == false) {         //do not drive if estop has been triggered
        if (mixedMode == false) {
          standardDrive();
        }
        else {
          mixedDrive();
        }

        if (SwB == 2000) {
          digitalWrite(D44, HIGH);
        }
        else {
          digitalWrite(D44, LOW);
        }
        if (SwD == 2000) {
          digitalWrite(D45, HIGH);
        }
        else {
          digitalWrite(D45, LOW);
        }

      }
      else {
        safeStartCheck();
      }
    }
    else {
      failsafe();

    }
  }

  //check to see if IBUS is alive
  if (Serial2.available()) {
    heartBeat = 0;
  }
  else {
    if (heartBeat < heartBeatTimeout + 10) { // don't overflow the counter
      heartBeat++;
    }
  }
}

void RX()
{
  //degub stuff---------------------
  //  Serial.print(F("Knob1:"));
  //  Serial.print(knob1);
  //  Serial.print(F("    Turn:"));
  //  Serial.print(steering);
  //  Serial.print(F("    Knob2:"));
  //  Serial.print(knob2);
  //  Serial.print(F("    Left:"));
  //  Serial.print(left);
  //  Serial.print(F("    Right:"));
  //  Serial.print(right);
  //  Serial.print(F("    Switch:"));
  //  Serial.print(SwC);
  //  Serial.print(F("  "));
  //  Serial.print(counter);
  //  Serial.print(F("  "));
  //  Serial.print(IBus.readChannel(9));
  //  Serial.println();

  steering = IBus.readChannel(0);
  right = IBus.readChannel(1);
  left = IBus.readChannel(2);
  lift = IBus.readChannel(3);
  knob1 = IBus.readChannel(4);
  knob2 = IBus.readChannel(5);
  SwA = IBus.readChannel(6);
  SwB = IBus.readChannel(7);
  SwC = IBus.readChannel(8);
  SwD = IBus.readChannel(9);

  //set mixed mode on controller---------
  if (SwA == 1000) {
    mixedMode = true;
  }
  else {
    mixedMode = false;
  }
  //----------------------------------------

  if (SwC == 2000) {
    autoBrake = false;
  }
  else {
    autoBrake = true;
  }

  ramp = map(knob1, 1000, 2000, rampMin, rampMax);    //11-20 = slow ramp, 21-80 = fast ramp
  maxSpeed = map(knob2, 1000, 2000, 0, 127);

  if (oldRamp != ramp) {      //Only set ramping if the value has changed
    ST.setRamping(ramp);      //The Sabertooth saves this to EEPROM
    oldRamp = ramp;
  }
}


//standard drive for two joystick control
void standardDrive()
{
  rightPower = map(right, 1000, 2000, maxSpeed * (-1) , maxSpeed);
  leftPower = map(left, 1000, 2000, maxSpeed * (-1), maxSpeed);

  liftSpeed = map(lift, 1000, 2000, -127, 127);


  if (autoBrake == true) {
    if (rightPower < 5 && rightPower > -5 && leftPower < 5 && leftPower > -5) {
      if (counter < autoBrakeThreshold + 10) {
        counter++;    //increment counter
      }
      if (counter > autoBrakeThreshold) {
        digitalWrite(brake, LOW);   //turn on brake
      }
    }
    else {
      digitalWrite(brake, HIGH);  //turn brake off
      counter = 0;
    }

  }
  else {
    digitalWrite(brake, HIGH);  //turn brake off
  }

  if (SwC == 1500) {         //if lift is enabled don't drive the wheels
    SyRen.motor(liftSpeed);
  }
  else {                        //lift is disabled so drive the wheels and turn off the lift
    ST.motor(1, rightPower);
    ST.motor(2, leftPower);
    SyRen.motor(0);              //disable lift
  }
}

//mixed drive for single joystick control
void mixedDrive() {

  rightPower = map(right, 1000, 2000, maxSpeed * (-1) , maxSpeed);
  leftPower = map(steering, 1000, 2000, maxSpeed * (-1), maxSpeed);

  liftSpeed = map(lift, 1000, 2000, -127, 127);


  if (autoBrake == true) {
    if (rightPower < 5 && rightPower > -5 && leftPower < 5 && leftPower > -5) {
      if (counter < autoBrakeThreshold + 10) {
        counter++;    //increment counter
      }
      if (counter > autoBrakeThreshold) {
        digitalWrite(brake, LOW);   //turn on brake
      }
    }
    else {
      digitalWrite(brake, HIGH);  //turn brake off
      counter = 0;
    }

  }
  else {
    digitalWrite(brake, HIGH);  //turn brake off
  }

  if (SwC == 1500) {         //if lift is enabled don't drive the wheels
    SyRen.motor(liftSpeed);
  }
  else {                        //lift is disabled so drive the wheels and turn off the lift
    ST.turn(rightPower);
    ST.drive(leftPower);
    SyRen.motor(0);             //disable lift
  }

}


//disables motors and turns on brake
void failsafe()
{
  ST.motor(1, 0);              //stop both motors
  ST.motor(2, 0);

  SyRen.motor(0);              //disable lift

  digitalWrite(brake, LOW);   //turn brake on

  if (digitalRead(estop) == LOW) {
    estopFlag = true;
  }

  failSafeFlag = true;

  if (ledEnable == true) {            //only turn on LED if enabled by dip switch
    //Turn LED Red--------------
    digitalWrite(LedR, LOW);
    digitalWrite(LedG, HIGH);
    digitalWrite(LedB, HIGH);
  }
  Serial.println("Failsafe");
}

//checks to see if controller input is in the neutral position before restarting
//prevents machine from moving after estop is reset
void safeStartCheck()
{
  //only reset estop flag if all joysticks are back in center position (+- 5)
  if (steering > 1495 && steering < 1505 && right > 1495 && right < 1505 && left > 1495 && left < 1505) {
    estopFlag = false;
  }

}
