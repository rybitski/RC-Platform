/*
  Wireless E Stop for scenery robot
  Adapted by Chris Rybitski 2018
  This code runs on the robot
  ------------------------------------------------------------------------------------
  Support:
  RC Platform v1.1 2019
  ------------------------------------------------------------------------------------

  ------------------------------------------------------------------------------------
  Update Log:
  11/14/19  Updated pins for RC Platform Board
  ------------------------------------------------------------------------------------
  Use at your own risk

  NRF24L01      Arduino
  CE       >     D8
  CSN      >     D10
  SCK      >     D13
  MO       >     D11
  MI       >     D12
  RO       >     Not used

  Blue LED   2
  Green LED  3
  Red LED    4
  Jumper     5
  Output1    6
  Output2    7
  Pot        A2

*/



#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

const int jumper = 5;
const int signalOUT = 6;
const int signalOUT2 = 7;
const int redLED = 4;
const int blueLED = 2;
const int greenLED = 3;
const int delayPin = A2;    //analog pin for threshold adjustment
bool LEDenable = true;      //controls LED output
int timeOUT = 0;            //stores count
int threshold = 500;          //decreases false positives, but increases reaction time



/*Radio*/
RF24 radio(8, 10);                                                      // SPI bus for E-Stop PCB are pins 8 and 10
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };           // Radio pipe addresses for the 2 nodes to communicate.
byte counter = 1;                                                       // A single byte to keep track of the data being sent back and forth


void setup() {

  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(jumper, INPUT);
  pinMode(signalOUT, OUTPUT);
  pinMode(signalOUT2, OUTPUT);


  //turn LED off by default-----------------------
  digitalWrite(redLED, HIGH); //turn off red LED
  digitalWrite(blueLED, HIGH);//turn off red LED
  digitalWrite(greenLED, HIGH); //turn off red LED

  Serial.begin(115200);
  printf_begin();


  // Setup and configure rf radio

  radio.begin();

  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0, 15);                // Smallest time between retries, max no. of retries(time, count) time in 250us multiples Max = 15
  radio.setPayloadSize(1);                // Here we are sending 1-byte payloads to test the call-response speed
  radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, and switch when writing
  radio.openReadingPipe(1, pipes[0]);
  radio.startListening();                 // Start listening
  radio.setPALevel(RF24_PA_MAX);          // Set PA to max  RF24_PA_MIN = 0, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging


  //check to see if the LEDs should be turned off
  if (digitalRead(jumper) == HIGH) {  //disables LED output on Robot if jumper is present
    LEDenable = false;
  }

  //allow user to adjust the threshold by adjusting a potentiometer. for each adjustment the arduino must be reset.
  threshold = (analogRead(delayPin) + 250);
  Serial.println(threshold);
}

void loop() {

  byte pipeNo;
  byte gotByte;

  while ( radio.available(&pipeNo)) {              // Dump the payloads until we've gotten everything
    radio.read( &gotByte, 1 );
    radio.writeAckPayload(pipeNo, &gotByte, 1 );
    timeOUT = 0;                                  //message received, reset counter
  }


  if (gotByte == 42 && timeOUT < threshold) {
    digitalWrite (signalOUT, HIGH);   //enable output
    digitalWrite (signalOUT2, HIGH);   //enable output
    gotByte = 1;    //reset message

    //only change the LED if the jumper was present at startup
    if (LEDenable == true) {
      digitalWrite(redLED, HIGH);       //turn off red LED
      digitalWrite(greenLED, LOW);          //turn on green LED
    }

  }
  else {
    if (timeOUT > threshold)        //only diasble if threshold is exceeded
    {
      digitalWrite (signalOUT, LOW);   //disable output
      digitalWrite (signalOUT2, LOW);   //disable output

      //only change the LED if the jumper was present at startup
      if (LEDenable == true) {
        digitalWrite(greenLED, HIGH);    //turn off green LED
        digitalWrite(redLED, LOW);      //turn on red LED
      }
    }

    delay(1);
  }
  timeOUT++;                       //increment counter
  //   Serial.println(timeOUT);
}
