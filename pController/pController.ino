#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <math.h>

// Create motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Assign motors to ports
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(1);

// Define sensor and motor pins
int leftIRPin = A0;
int rightIRPin = A1;

// Initialize variables
const float P = .15;
int leftIR;
int rightIR;
int leftAdjust = 800;
int rightAdjust = 800;
int adjustCoeff = 1.092;
int leftMotorSpeed;
int rightMotorSpeed;
int baseSpeed = 30;

void setup() {
  // Open serial
  Serial.begin(9600);
  
  AFMS.begin();
}

void loop() {
//  // Define IR leftReadings
  leftIR = analogRead(leftIRPin)-leftAdjust;
  rightIR = analogRead(rightIRPin)*1.06-rightAdjust;

  // Define motor speeds
  leftMotorSpeed = baseSpeed + P*rightIR;
  rightMotorSpeed = baseSpeed + P*leftIR;
  
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
  myLeftMotor->run(FORWARD);
  myRightMotor->run(FORWARD);
  
  Serial.println(String(leftIR)+" "+String(rightIR)+" "+String(leftMotorSpeed)+" "+String(rightMotorSpeed));
  
  delay(10);
}
