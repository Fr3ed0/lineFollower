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

// Define variables needed for serialEvent()
String inputString = "";          //a string that reads in the inputted characters one by one
boolean stringComplete = false;   //set to true when a new line character is detected and the string is over

// Initialize variables
const float P = 1;
const float I = 0;
const float D = 0;
const byte leftTarget = 10;
const byte rightTarget = 10;
int leftIR;
int rightIR;
int leftMotorSpeed;
int rightMotorSpeed;

byte currentIndex = 0;
const int numReadings = 100;
byte leftReadings[numReadings]; //array of saved analog read inputs
byte rightReadings[numReadings]; //array of saved analog read inputs
int leftReadingSum = 0; //use for integral
int rightReadingSum = 0; //use for integral
float leftAverage = 0.0;
float rightAverage = 0.0;
byte leftError;
byte rightError;
byte leftErrors[numReadings];
byte rightErrors[numReadings];
int leftIntegral;
int rightIntegral;
int timeDelay = 100;

void setup() {
  // Open serial
  Serial.begin(9600);
  inputString.reserve(200);   //save some space on the Arduino for the user inputted string
  
  // Fill arrays with zeroes
  for (byte currentReading = 0; currentReading < numReadings; currentReading ++){
    leftReadings[currentReading] = 0;
    rightReadings[currentReading] = 0;
    leftErrors[currentReading] = 0;
    rightErrors[currentReading] = 0;
  }
}

void loop() {
  // Define IR leftReadings
  leftIR = map(analogRead(leftIRPin), 60, 930, 0, 255);
  rightIR = map(analogRead(rightIRPin), 60, 930, 0, 255);
  
  // Subtract old reading from sums
  leftReadingSum -= leftReadings[currentIndex];
  rightReadingSum -= rightReadings[currentIndex];
  
  leftError = leftTarget- leftReadings[currentIndex];
  rightError = rightTarget - rightReadings[currentIndex];
  
  leftErrorSum -= leftError;
  rightErrorSum -= rightError;
  
  // Add readings to arrays
  leftReadings[currentIndex] = leftIR;
  rightReadings[currentIndex] = rightIR;
  
  // Add new reading to sums
  leftReadingSum += leftReadings[currentIndex];
  rightReadingSum += rightReadings[currentIndex];
  
  // Increment array positions
  currentIndex ++;
  currentIndex ++;
  
  // Revert to beginning of array if index out of bounds
  if (currentIndex >= numReadings){
      currentIndex = 0;
  }
  if (currentIndex >= numReadings){
      currentIndex = 0;
  }
  
  // Calculate reading averages
  leftAverage = leftReadingSum/float(numReadings);
  rightAverage = rightReadingSum/float(numReadings);
  
  // Calculate error
  leftError = leftTarget - leftReadings[currentIndex];
  rightError = rightTarget - rightReadings[currentIndex];
  
  // Calculate integral
  leftIntegral = leftErrorSum * (timeDelay/1000.0);
  rightIntegral = rightErrorSum * (timeDelay/1000.0);
  
  // Calculate past reading indices
  if (currentIndex - 1 >= 0)
    pastIndex = currentIndex - 1;
  else
    pastIndex = currentIndex -1 + 10;
  if (currentIndex - 2 >= 0)
    pastPastIndex = currentIndex - 2;
  else
    pastPastIndex = currentIndex -2 + 10;
  
  // Calculate derivatives
  leftDeriv = (leftReading[pastIndex] - leftReading[pastPastIndex]) * (timeDelay/1000.0);
  rightDeriv = (rightReading[pastIndex] - rightReading[pastPastIndex]) * (timeDelay/1000.0);

  //Get user input for P
  serialEvent()
  if (stringComplete){
    Serial.println(inputString);
    P = inputString;
    inputString = "";         //reset the string to empty
    stringComplete = false;   //reset the complete boolean to false
  }
  
  // Define motor speeds
  leftMotorSpeed = P*rightError + I*rightIntegral + D*rightDeriv;
  rightMotorSpeed = P*leftError + I*leftIntegral + D*leftDeriv;
  
  if (leftMotorSpeed > 255)
    leftMotorSpeed = 255;

  if (rightMotorSpeed > 255)
    rightMotorSpeed = 255;

  // Set motor direction
  myLeftMotor->run(FORWARD);
  myRightMotor->run(FORWARD);

  // Set motor speed
  myLeftMotor->setSpeed(leftMotorSpeed);
  myRightMotor->setSpeed(rightMotorSpeed);
  
  delay(timeDelay);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
