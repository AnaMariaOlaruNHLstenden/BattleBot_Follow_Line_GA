//=======================================================================//
//.............................LIBRARIES.................................//
//=======================================================================//

#include <Arduino.h> // Arduino library needed for Visual Studio(PlatformIO)
#include <Adafruit_NeoPixel.h> // Neopixel Library to control the neopixels

//=======================================================================//
//.....................PREDECLARATION OF FUNCTIONS.......................//
//=======================================================================//

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void testLights();
void startLights();
void endLights();
void leftLights();
void rightLights();
void neutralLights();
void setMotors(int LFFwd, int LBBwd, int RFFwd, int RBBwd);
void driveForward(int leftSpeed, int rightSpeed);
void driveBackward(int leftSpeed, int rightSpeed);
void driveLeft(int leftSpeed, int rightSpeed);
void driveRight(int leftSpeed, int rightSpeed);
void driveStop();
void defaultLineSensor();
void scanBlackBox_START();
void scanBlackBox_END();
void scanBlackBox();
void fullScan();
void distanceSensor();
void distanceReader();
void gripToggle();
void servo(int pulse);

//=======================================================================//
//................................I/O PINS...............................//
//=======================================================================//
#define PIN 8 // Neopixel pin NI(Input)
#define NUMPIXELS 4 // There are 4 neopixels on the board

const int echoPin = 4; // Echo sensor echo pin
const int triggerPin = 5; // Echo sensor trigger pin


const int gripperPin = 12; // Gripper pin GR


const int LB = 11; // Motor Left Backwards pin A1 LEFT WOBBLY
const int LF = 10; // Motor Left Forwards pin A2
const int RB = 9; // Motor Right Backwards pin B1 RIGHT WEAK
const int RF = 6; // Motor Right Forwards pin B2

const int motorPulseLeft = 2; // Motor pin R1
const int motorPulseRight = 3; // Motor pin R2 

const int numberOfSensors = 8;
int lineSensor[numberOfSensors] = {A5,A4,A7,A3,A2,A6,A1,A0} ; // Linesensor pins
/*=============================================
A0 = D8      A2 = D5      A4 = D2      A7 = D3 
A1 = D7      A3 = D4      A5 = D1      A6 = D6
===============================================*/

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800); // Neopixel needed code from library

//=======================================================================//
//...............................VARIABLES...............................//
//=======================================================================//

//-------TIMED AND BOOL-------//

 // Counting time unsing unsigned long to store the huge about of time that is counted
 unsigned long startMillis; 
 unsigned long previoustimeDS_2 = 0; //Previous time from the past loop
 unsigned long previoustimeDS_1 = 0;
 unsigned long previoustimeLS_1 = 0;

 unsigned long interval_1 = 100; //Intervals for multitasking
 unsigned long interval_2 = 200;
 unsigned long interval_3 = 30;

 bool startTrigger = false; // Flag Verification
 bool startRace = false;
 bool endRace = false;

//-------ULTRASONIC-------//
const int maxDistance = 20; // 20 cm maximum distance to leave room for error
const int startDistance = 15; // 15 cm maximum distance from the flag
float distance, duration; // declared 2 floats in 1 line for organization purposes
const int echoInterval = 245;

//--------GRIPPER-------//
int gripOpen = 2000; // pulse length servo open
int gripClosed = 1160; // pulse length servo closed
int servoInterval = 20; // time between pulse

//--------MOVEMENT-------//
const int leftSlowSpeed = 105; // Slowest speed left wheel
const int rightSlowSpeed = 125; // Slowest speed right
const int speedTurns = 55; // Adding speed for turns
const int speedSharpT = 60; // Adding speed for sharp turns
const int speedOneWay = 50; // Adding speed for one direction not turns
const int startSpeed = 40; // Adding speed for start
const int additionalSpeed = 70;

//--------SENSOR---------//
int lineValues[numberOfSensors];
int maxSensorValue = 0;
const int MAX_BLACK = 980;
const int MIN_BLACK = 900;
const int MAX_GRAY = 700;
const int MIN_GRAY = 600;
const int MIN_WHITE = 500;
const int MAX_WHITE = 400;



int lineCount = 0;

//=======================================================================//
//.................................SETUP.................................//
//=======================================================================//

void setup() {
  strip.begin(); // Initialize neopixels
  strip.show(); // Set neopixel on off
  Serial.begin(9600); // Start serial monitoring on 9645

//--------SETUP THE PIN_MODES---------//
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

  //--------SETUP FUNCTIONS THAT START AND FIRE ONCE---------//

  digitalWrite(gripperPin, LOW); //To open the Gripper
  servo(gripOpen); //Gripper is open

}

//=======================================================================//
//.............................LOOP FUNCTION.............................//
//=======================================================================//

void loop() {

  //Start trigger waits for the flag to be lift up
  if (startTrigger == false)
  {
    distanceReader();
    startLights();
    while (distance < startDistance)
    {
      driveStop();
      servo(gripOpen); 
      Serial.println(distance);
      distanceReader();
      if (distance > startDistance)
      {
        break;
      }
      
    } 
    startTrigger = true;
    Serial.println("left while loop");
    setMotors(255, 0, 255, 0);
    delay(50);
    setMotors(140, 0, 160, 0);
    delay(1350);
  } 
  
  
    bool lineScanInProgress = false; // Flag to indicate if line scanning is in progress
    unsigned long currentMillis = millis(); // Get the current time
    scanBlackBox();
    // Check if black color is detected for more than 5 milliseconds
    static unsigned long timer;
    if (currentMillis > timer)
    {
      if (lineValues[0] >= MAX_BLACK && !lineScanInProgress && lineValues[7] >= MAX_BLACK && !lineScanInProgress) {
        lineScanInProgress = true; // Set flag to indicate line scanning is in progress
        lineCount++; // Add to the counter
      }
      timer = currentMillis + 50;
    }
    
    //Start sequence of grabbing the object
    if (lineScanInProgress && lineCount >= 4) 
      {
          if (startRace == false)
          {
            scanBlackBox();
            while (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK )
            {
              scanBlackBox_START();

              if (startRace == true)
              {
                break;
              }
            }
          }
        lineScanInProgress = false;
    }
    
    defaultLineSensor();
    distanceSensor();
    
    //End sequence of dropping the object
  if (!endRace && startRace)
   {
    fullScan();
    if (lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK) 
        {
          //This code checks twice if it's the black square or not
            setMotors(140, 0, 160, 0);
            pulseIn(2,HIGH,300UL);
            pulseIn(3,HIGH,300UL);
            pulseIn(2,HIGH,300UL);
            pulseIn(3,HIGH,300UL);
            pulseIn(2,HIGH,300UL);
            pulseIn(3,HIGH,300UL);
            delay(25);
            fullScan();
        }
        if (lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK)
        {
              scanBlackBox_END();
        }
    }
    
       

//TO DO : IMPLEMENT NEOPIXEL STATES AND END

}


//=======================================================================//
//...............................FUNCTIONS...............................//
//=======================================================================//

//=======================================================================//
//................................MOTORS.................................//
//=======================================================================//

void setMotors(int LFFwd, int LBBwd, int RFFwd, int RBBwd) {
  // Sets the speed of all the wheels by entering parameters
  analogWrite(LF, LFFwd);
  analogWrite(LB, LBBwd);
  analogWrite(RF, RFFwd);
  analogWrite(RB, RBBwd);
}

void driveForward(int leftSpeed, int rightSpeed) {
  neutralLights();
  //setMotors(219, 0, 254, 0); // Set speeds of the motor; LeftForward - LeftBackward - RightForward - RightBackward
  //Slowest LF Speed is 160
  //Slowest RF Speed is 196
  setMotors(leftSpeed, 0, rightSpeed, 0);
}

void driveBackward(int leftSpeed, int rightSpeed) {
  //setMotors(0, 219, 0, 254);
  setMotors(0, leftSpeed, 0, rightSpeed);
}

void driveRight(int leftSpeed, int rightSpeed) {
  rightLights();
  //setMotors(219, 0, 0, 254);
  setMotors(leftSpeed, 0, 0, rightSpeed);
}

void driveLeft(int leftSpeed, int rightSpeed) {
  leftLights();
  //setMotors(0, 219, 254, 0);
  setMotors(0, leftSpeed, rightSpeed, 0);
}

void driveStop() {
  setMotors(0, 0, 0, 0);
}

//=======================================================================//
//................................LINE SENSOR............................//
//=======================================================================//

void defaultLineSensor() {
  // Read reflection sensor values
  fullScan();

  // Serial.print("Reflection Sensor Values: ");
  // for (int i = 0; i < numberOfSensors; i++) {
  //   Serial.print(i);
  //   Serial.print(" ");
  //   Serial.print(lineValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
if (previoustimeLS_1 < millis())
{
 for (int i = 0; i < numberOfSensors; i++) {
    if (lineValues[i] > maxSensorValue) {
      maxSensorValue = lineValues[i];
    }
  } 
}
previoustimeLS_1 = 20UL + millis();
  
 // Use thresholds to determine the behavior Slowestd on the maximum sensor value
  if (maxSensorValue >= MAX_BLACK) {

    if (lineValues[3] >= MIN_BLACK || lineValues[4] >= MIN_BLACK ){
      driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed); //We start with slowest and then modify the speed to a decent speed
      Serial.print("forward");
      Serial.print(" ");
    }
    else if ( lineValues[2] >= MAX_BLACK){
      driveRight(leftSlowSpeed + speedTurns + additionalSpeed,rightSlowSpeed + speedTurns + additionalSpeed);
      Serial.print("right1111");
      Serial.print(" ");
    }
    else if (lineValues[5] >= MAX_BLACK){
      driveLeft(leftSlowSpeed + speedTurns + additionalSpeed,rightSlowSpeed + speedTurns + additionalSpeed);
      Serial.print("left111");
      Serial.print(" ");
    }
    else if (lineValues[1] >= MIN_BLACK){
      driveRight(leftSlowSpeed + speedSharpT + additionalSpeed,rightSlowSpeed + speedSharpT + additionalSpeed);
      Serial.print("right222");
      Serial.print(" ");
    }
    else if (lineValues[6] >= MIN_BLACK){
      driveLeft(leftSlowSpeed + speedSharpT + additionalSpeed,rightSlowSpeed + speedSharpT + additionalSpeed);
      Serial.print("left222");
      Serial.print(" ");
    }  
  } 
}

void scanBlackBox()
{
   for (int i = 0; i < 2; i++) {
    lineValues[0] = analogRead(lineSensor[0]);
    lineValues[7] = analogRead(lineSensor[7]);
  }  
}

void fullScan()
{
  // Read reflection sensor values
  for (int i = 0; i < numberOfSensors; i++) 
  {
    lineValues[i] = analogRead(lineSensor[i]);
  }  
}

void scanBlackBox_START()
{
  if (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK )
  {
      static bool state;
    Serial.print("I see le box");
      Serial.print(" ");
       
        if (state == false) {
            Serial.print("dead");
            Serial.print(" ");
          
          for (int i = 0; i < 10; i++)
          {
            servo(gripClosed);
          }
          
          startRace = true;
        }
      }
      driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
      delay(1350);
      driveForward(leftSlowSpeed + startSpeed,rightSlowSpeed + startSpeed);
   defaultLineSensor();
}

void scanBlackBox_END()
{

  fullScan();

  if(lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK)
  {
      static bool state;
    Serial.print("I see le box");
      Serial.print(" ");

        if (state == false) {
          Serial.print("dead");
          Serial.print(" ");
          servo(gripOpen);
          endRace = true;
        }
  }

  if (endRace)
    {
      driveBackward(leftSlowSpeed + speedOneWay,rightSlowSpeed + speedOneWay);
      delay(400);
    }
    
    while (endRace)
    {
      endLights();
      driveStop();
    }  
}

//=======================================================================//
//............................ULTRASONIC SENSOR..........................//
//=======================================================================//

void distanceReader()
{
  digitalWrite(triggerPin, LOW); // Reset pin
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH); // High pulses for 10 ms
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    

    duration = pulseIn(echoPin, HIGH); // Reads pins

    distance = (duration / 2) * 0.0343; // 343 m/s per second as speed of sound
}

void distanceSensor(){
static unsigned long timer;
 if (millis() > timer) 
  {

    distanceReader();
    
    if (distance <= maxDistance && lineCount >= 4)
      {
        Serial.println("Avoid Object"); //This function makes sure that anything closer than 20CM it will avoid it
        
          Serial.println("right");
          Serial.println(" ");
          driveRight(leftSlowSpeed + speedTurns, rightSlowSpeed);
          delay(800);

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay, rightSlowSpeed + speedOneWay);
          delay(800);

         Serial.println("left");
          Serial.println(" ");
          driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
          delay(800);

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay, rightSlowSpeed + speedOneWay);
          delay(1000);
       
          Serial.println("left");
          Serial.println(" ");
          driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
          delay(800);

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay,rightSlowSpeed + speedOneWay);
          delay(50);

        if (lineValues[3] >= MIN_BLACK) // This is to search for the line in case the turn needs to be longer
        {
          defaultLineSensor();
        }
        else
        {
          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay,rightSlowSpeed + speedOneWay);
          delay(750);

            Serial.println("left");
          Serial.println(" ");
          driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
          delay(800);
        }
          driveForward(leftSlowSpeed + speedOneWay,rightSlowSpeed + speedOneWay);
          
        defaultLineSensor();
      }
    else
      {
        Serial.print(distance);
        Serial.println(" cm");
        defaultLineSensor();
      }
       timer = millis() + 200;
    }
}

//=======================================================================//
//................................GRIPPER................................//
//=======================================================================//

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

//=======================================================================//
//...............................NEO PIXELS..............................//
//=======================================================================//

//  Function to set the color of a single neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red,green,blue));
  strip.show(); // Set neopixel on off
}

void testLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 222, 222, 0); //left back
  setPixelColor(1, 222, 222, 0); //right back
  setPixelColor(2, 222, 222, 0); //right front
  setPixelColor(3, 222, 222, 0); //left front
}

void startLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 255, 10, 10); //left back
  setPixelColor(1, 255, 10, 10); //right back
  setPixelColor(2, 255, 10, 10); //right front
  setPixelColor(3, 255, 10, 10); //left front
}

void endLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 255, 94, 36); //left back
  setPixelColor(1, 255, 94, 36); //right back
  setPixelColor(2, 255, 94, 36); //right front
  setPixelColor(3, 255, 94, 36); //left front
}
void leftLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 239, 94, 255); //left back
  setPixelColor(1, 222, 222, 0); //right back
  setPixelColor(2, 222, 222, 0); //right front
  setPixelColor(3, 239, 94, 255); //left front
}
void rightLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 222, 222, 0); //left back
  setPixelColor(1, 94, 196, 255); //right back
  setPixelColor(2, 94, 196, 255); //right front
  setPixelColor(3, 222, 222, 0); //left front
}
void neutralLights() {  // Set the color of the neopixels by pixel number and rgb value
  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 222, 222, 0); //left back
  setPixelColor(1, 222, 222, 0); //right back
  setPixelColor(2, 222, 222, 0); //right front
  setPixelColor(3, 222, 222, 0); //left front
}