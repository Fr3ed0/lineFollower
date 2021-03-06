#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <math.h>

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Assign motors to ports
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(1);

// Define sensor pins
int leftIRPin = A0;
int rightIRPin = A1;

// Initialize state variables
int leftIR;
int rightIR;
int leftMotorSpeed;
int rightMotorSpeed;

// Initialze control loop variables
float P = .8;
int baseAdjust = 800;
float adjustCoeff = 1.3;
int baseSpeed = 140;

// Initialize variables for serial input and output
char letter;
long signed int value;
#define PC_BAUD 250000
HardwareSerial & pcSer = Serial;

void setup() {
  // Open input serial
//  Serial.begin(9600);
//  Serial.setTimeout(5);       //time out for parseInt() and things that wait for bytes

  // Open output serial
  pcSer.begin(PC_BAUD);

  // Begin motor control
  AFMS.begin();
}

void loop() {
  // Define IR leftReadings
  leftIR = analogRead(leftIRPin) - baseAdjust;
  rightIR = (analogRead(rightIRPin) - baseAdjust) * adjustCoeff;

//  //Code that reads inputted values for P I and D; input as p98 (no space)
//  if (Serial.available() > 0) {
//    letter = Serial.read();
//    value = Serial.parseInt();
//    if (value != 0) {
//      if (letter == 'P') {
//        P = value/10.0;
//        Serial.println(P);
//      }
//  }
//}

  // Define motor speeds
  leftMotorSpeed = baseSpeed - P * leftIR;
  rightMotorSpeed = baseSpeed - P * rightIR;

  // Bound motor speeds between 0 and 255
  if (leftMotorSpeed > 255)
    leftMotorSpeed = 255;
  else if (leftMotorSpeed < 0)
    leftMotorSpeed = 0;

  if (rightMotorSpeed > 255)
    rightMotorSpeed = 255;
  else if (rightMotorSpeed < 0)
    rightMotorSpeed = 0;

  // Set motor speed
  myLeftMotor->setSpeed(leftMotorSpeed);
  myRightMotor->setSpeed(rightMotorSpeed);

  // Set motor direction
  myLeftMotor->run(BACKWARD);
  myRightMotor->run(BACKWARD);

//  Serial.println(String(leftIR) + " " + String(rightIR) + " " + String(leftMotorSpeed) + " " + String(rightMotorSpeed));
  sendToSerial(leftIR, rightIR, leftMotorSpeed, rightMotorSpeed);

  delay(10);
}

void sendToSerial(int lIR, int rIR, int lMotor, int rMotor) {
  // Send IR and motor values through serial delimited by commas
  pcSer.print(lIR);
  pcSer.print(", ");
  pcSer.print(rIR);
  pcSer.print(", ");
  pcSer.print(lMotor);
  pcSer.print(", ");
  pcSer.println(rMotor);
}
