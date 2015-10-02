#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <math.h>

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Assign motors to ports
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(2);
//  myMotor->setSpeed(150);
//  myMotor->run(FORWARD);
//  myMotor->run(RELEASE);

// Define sensor and motor pins
int leftIRPin = A0;
int rightIRPin = A1;

// Initialize variables
const float P = 1;
const float I = 0;
const float D = 0;
const byte leftTarget = 10;
const byte rightTarget = 10;
int leftIR;
int rightIR;
byte leftMotorSpeed;
byte rightMotorSpeed;

const int leftNumReadings = 100; 
const int rightNumReadings = 100;
int leftReadings[leftNumReadings]; //array of saved analog read inputs
int leftReadIndex = 0; //Index of leftReadings
int leftSum = 0; //use for integral
int rightReadings[leftNumReadings]; //array of saved analog read inputs
int rightReadIndex = 0; //Index of leftReadings
int rightSum = 0; //use for integral
int leftAverage = 0;
int rightAverage = 0;
byte


void setup() {
  // Open serial
  Serial.begin(9600);
  for (int leftThisReading = 0; leftThisReading < leftNumReadings; leftThisReading ++){
    leftReadings[leftThisReading] = 0;
  }
  for (int rightThisReading = 0; rightThisReading < rightNumReadings; rightThisReading ++){
    rightReadings[rightThisReading] = 0;
  }
}

void loop() {
  // Define IR leftReadings
  leftIR = map(analogRead(leftIRPin), 60, 930, 100, 255);
  rightIR = map(analogRead(rightIRPin), 60, 930, 100, 255);
  
  // Add readings to arrays
  leftReadings[leftReadIndex] = leftIR;
  rightReadings[rightReadIndex] = rightIR;
  
  // Subtract previous point
  leftSum = leftSum - leftReadings[leftReadIndex];
  rightSum = rightSum - rightReadings[rightReadIndex];
  
  // Advancing in the array
  leftReadIndex = leftReadIndex + 1;
  rightReadIndex = rightReadIndex + 1;
  
  // Revert to beginning of array if index out of bounds
  if (leftReadIndex >= leftNumReadings){
      leftReadIndex = 0; //wraps to beginning
  }
  if (rightReadIndex >= rightNumReadings){
      rightReadIndex = 0; //wraps to beginning
  }
  
  // Calculate averages
  leftAverage = leftSum/leftNumReadings;
  rightAverage = rightSum/rightNumReadings;
  delay(1);

  // Define left motor speed
  if (round(rightIR * P <= 255)) {
    leftMotorSpeed = round(rightIR * P);
  } else {
    leftMotorSpeed = 255;
  }

  // Define right motor speed
  if (round(leftIR * P <= 255)) {
    rightMotorSpeed = round(leftIR * P);
  } else {
    rightMotorSpeed = 255;
  }

  // Set motor direction
  myLeftMotor->run(FORWARD);
  myRightMotor->run(FORWARD);

  // Set motor speed
  myLeftMotor->setSpeed(leftMotorSpeed);
  myRightMotor->setSpeed(rightMotorSpeed);
}



