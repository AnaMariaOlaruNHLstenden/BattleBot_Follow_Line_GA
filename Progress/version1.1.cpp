//.............................LIBRARIES..................................

#include <Arduino.h> // Arduino library needed for Visual Studio(PlatformIO)
#include <Adafruit_NeoPixel.h> // Neopixel Library to control the neopixels

//....................PREDECLARATION OF FUNCTIONS.........................

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void testLights();
void setMotors(int LFFwd, int LBBwd, int RFFwd, int RBBwd);
void driveForward(int leftSpeed, int rightSpeed);
void driveBackward(int leftSpeed, int rightSpeed);
void driveLeft(int leftSpeed, int rightSpeed);
void driveRight(int leftSpeed, int rightSpeed);
void driveStop();
void defaultLineSensor();
void distanceSensor();
void gripToggle();
void servo(int pulse);

//.............................I/O PINS...................................

#define PIN 8 // Neopixel pin NI(Input)
#define NUMPIXELS 4 // There are 4 neopixels on the board

const int echoPin = 4; // Echo sensor echo pin
const int triggerPin = 5; // Echo sensor trigger pin
const int maxDistance = 20; // 20 cm maximum distance to leave room for error
float distance, duration; // declared 2 floats in 1 line for organization purposes
const int echoInterval = 250;

const int gripperPin = 12; // Gripper pin GR
int gripOpen = 1600; // pulse length servo open
int gripClosed = 930; // pulse length servo closed
int servoInterval = 20; // time between pulse
int gripperToggle = 1000; // toggle gripper every second

const int LB = 11; // Motor Left Backwards pin A1 LEFT WOBBLY
const int LF = 10; // Motor Left Forwards pin A2
const int RB = 9; // Motor Right Backwards pin B1 RIGHT WEAK
const int RF = 6; // Motor Right Forwards pin B2

const int motorPulseLeft = 2; // Motor pin R1
const int motorPulseRight = 3; // Motor pin R2 

const int numberOfSensors = 8;
int lineSensor[numberOfSensors] = {A5,A4,A7,A3,A2,A6,A1,A0} ; // Linesensor pins
int lineValues[numberOfSensors];
int maxSensorValue = 0;
/*

A0 = D8      A2 = D5      A4 = D2      A6 = D6 
A1 = D7      A3 = D4      A5 = D1      A7 = D3
*/
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800); // Neopixel needed code from library

//................................SETUP....................................

void setup() {
  strip.begin(); // Initialize neopixels
  strip.show(); // Set neopixel on off
  Serial.begin(9650); // Start serial monitoring on 9650
  // Setup the pinModes
  pinMode(LF, OUTPUT);  // Specify the LeftForward motor to be Output
  pinMode(LB, OUTPUT);  // Specify the LeftBackward motor to be Output
  pinMode(RF, OUTPUT);  // Specify the RightForward motor to be Output
  pinMode(RB, OUTPUT);  // Specify the RightBackward motor to be Output
  pinMode(gripperPin, OUTPUT);  // Specify the gripperpin to be Output
  pinMode(triggerPin, OUTPUT);  // Specify the triggerPin to be Output
  pinMode(echoPin, INPUT);  // Specify the echoPin to be Input
  pinMode(motorPulseLeft, INPUT); // Specify the motorPulseLeft to be Input
  pinMode(motorPulseRight,INPUT); // Specify the motorPulseRight to be Input
   for(int i = 0;i<=7;i++) 
  {
    pinMode(lineSensor[i], INPUT);
  }
  // Setup functions that start and fire onces
  testLights(); // Call the function to set the neopixel lights
}

//.............................LOOP FUNCTION...............................

void loop() {
  defaultLineSensor();
  distanceSensor();
}

//..............................Functions...................................

//  Function to set the color of a single neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red,green,blue));
 // strip.show(); // Set neopixel on off
}

void testLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 222, 222, 0); //left back
  setPixelColor(1, 222, 222, 0); //right back
  setPixelColor(2, 222, 222, 0); //right front
  setPixelColor(3, 222, 222, 0); //left front
}
//..............................MOTORS......................................

void setMotors(int LFFwd, int LBBwd, int RFFwd, int RBBwd) {
  // Sets the speed of all the wheels by entering parameters
  analogWrite(LF, LFFwd);
  analogWrite(LB, LBBwd);
  analogWrite(RF, RFFwd);
  analogWrite(RB, RBBwd);
}

void driveForward(int leftSpeed, int rightSpeed) {
  setMotors(219, 0, 254, 0); // Set speeds of the motor; LeftForward - LeftBackward - RightForward - RightBackward
  //Base LF Speed is 160
  //Base RF Speed is 196
  setMotors(leftSpeed, 0, rightSpeed, 0);
}

void driveBackward(int leftSpeed, int rightSpeed) {
  setMotors(0, 219, 0, 254);
  setMotors(0, leftSpeed, 0, rightSpeed);
}

void driveRight(int leftSpeed, int rightSpeed) {
  setMotors(219, 0, 0, 254);
  setMotors(leftSpeed, 0, 0, rightSpeed);
}

void driveLeft(int leftSpeed, int rightSpeed) {
  setMotors(0, 219, 254, 0);
  setMotors(0, leftSpeed, rightSpeed, 0);
}

void driveStop() {
  setMotors(0, 0, 0, 0);
}

//................................LINE SENSOR...................................

void defaultLineSensor() {
  // Read reflection sensor values
  for (int i = 0; i < numberOfSensors; i++) {
    lineValues[i] = analogRead(lineSensor[i]);
  }  

  Serial.print("Reflection Sensor Values: ");
  for (int i = 0; i < numberOfSensors; i++) {
    Serial.print(i);
    Serial.print(" ");
    Serial.print(lineValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  for (int i = 0; i < numberOfSensors; i++) {
    if (lineValues[i] > maxSensorValue) {
      maxSensorValue = lineValues[i];
    }
  }
 // Use thresholds to determine the behavior based on the maximum sensor value
  if (maxSensorValue >= 900) {

    if (600 >= lineValues[4] >= 900 || lineValues[3] >= 900)
      driveForward(105+40, 123+40);

    else if (lineValues[0] >= 900 || lineValues[1] >= 900  || lineValues[2] >= 900 )
      driveRight(105+65,123+65);

    else if (lineValues[5] >= 900 || lineValues[6] >= 900 || lineValues[7] >= 900)
      driveLeft(105+65,123+65);
  } 
  else if (maxSensorValue >= 700) {
    if (lineValues[0] >= 700 || lineValues[7] >= 700)
      driveForward(105+40, 123+40);

    else if (lineValues[3] >= 700 || lineValues[2] >= 700 ||  lineValues[1] >= 700)
      driveRight(105+65,123+65);

    else if (lineValues[4] >= 700 || lineValues[5] >= 700 || lineValues[6] >= 700)
      driveLeft(105+65,123+65);
  } 
}

//................................ULTRASONIC SENSOR..................................

void distanceSensor(){
static unsigned long timer;
  if (millis() > timer)
  {
    digitalWrite(triggerPin, LOW); // Reset pin
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH); // High pulses for 10 ms
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    duration = pulseIn(echoPin, HIGH); // Reads pins

    distance = (duration / 2) * 0.0343; // 343 m/s per second as speed of sound

    if (distance <= maxDistance)
      {
        Serial.println("Avoid Object");
      }
    else
      {
        Serial.print(distance);
        Serial.println(" cm");
      }
      timer = millis() + echoInterval;
  }
}

//................................GRIPPER..................................



void gripToggle(){
 static unsigned long timer;
  static bool state;
  if (millis() > timer) {
    if (state == true) {
      servo(gripOpen);
      state = false;
    } else {
      servo(gripClosed);
      state = true;
    }
    timer = millis() + gripperToggle;
  }
}

void servo(int pulse) {
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(gripperPin, LOW);
    timer = millis() + servoInterval;
  }
}

//................................MILLIS...................................

//TODO: AVOID OBSTACLES AND GRAB OBJECT FOR THE WHOLE WORKING TIME, FINISH/START PART