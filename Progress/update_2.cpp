//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░░ █ █▄▄ █▀█ ▄▀█ █▀█ █ █▀▀ █▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▄▄ █ █▄█ █▀▄ █▀█ █▀▄ █ ██▄ ▄█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

#include <Arduino.h> // Arduino library needed for Visual Studio(PlatformIO)
#include <Adafruit_NeoPixel.h> // Neopixel Library to control the neopixels

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░█▀█ █▀█ █▀▀ █▀▄ █▀▀ █▀▀ █░░ █▀▀ █▀█ ▄▀█ ▀█▀ █ █▀█ █▄░█   █▀█ █▀▀   █▀▀ █░█ █▄░█ █▀▀ ▀█▀ █ █▀█ █▄░█ █▀░░░░░░//
//░░░░░░█▀▀ █▀▄ ██▄ █▄▀ ██▄ █▄▄ █▄▄ ██▄ █▀▄ █▀█ ░█░ █ █▄█ █░▀█   █▄█ █▀░   █▀░ █▄█ █░▀█ █▄▄ ░█░ █ █▄█ █░▀█ ▄█░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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
void scanBlackBox(); // Sensors 0 and 7
void fullScan(); // All Sensors
void distanceSensor();
void distanceReader();
void servo(int pulse);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█ ░░▄▀ █▀█   █▀█ █ █▄░█ █▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█ ▄▀░░ █▄█   █▀▀ █ █░▀█ ▄█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_RGB + NEO_KHZ800); // Neopixel needed code from library
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░█ ▄▀█ █▀█ █ ▄▀█ █▄▄ █░░ █▀▀ █▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▀▄▀ █▀█ █▀▄ █ █▀█ █▄█ █▄▄ ██▄ ▄█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//-------TIMED AND BOOL-------//

 // Counting time unsing unsigned long to store the huge about of time that is counted
 unsigned long previoustimeLS_1 = 0;//Previous time from the past loop

 bool startTrigger = false; // Flag Verification
 bool startRace = false; // Start Verification
 bool endRace = false; // End Verification

//-------ULTRASONIC-------//
const int maxDistance = 15; // 20 cm maximum distance to leave room for error
const int startDistance = 15; // 15 cm maximum distance from the flag
float distance, duration; // declared 2 floats in 1 line for organization purposes
const int echoInterval = 100; // Time between seeing the object and calculating the distance

//--------GRIPPER-------//
int gripOpen = 2000; // pulse length servo open
int gripClosed = 1160; // pulse length servo closed
int servoInterval = 20; // time between pulse

//--------MOVEMENT-------//
const int leftSlowSpeed = 105; // Slowest speed left wheel
const int rightSlowSpeed = 129; // Slowest speed right wheel
const int backwardsSpeedLeft = 0;// Backwards speed left Wheel
const int backwardsSpeedRight = 0;// Backwards speed right Wheel
const int speedTurns = 55; // Adding speed for turns
const int speedSharpT = 60; // Adding speed for sharp turns
const int speedOneWay = 50; // Adding speed for one direction not turns
const int startSpeed = 40; // Adding speed for start
const int additionalSpeed = 60; // Additional modifiable speed  to methods who have speed but could make complications


//--------SENSOR---------//
int lineValues[numberOfSensors];
int maxSensorValue = 0; // Setting Gate
const int MAX_BLACK = 980; // The Max Value that is easily reached
const int MIN_BLACK = 900;// The Min Value of the black
const int GREY = 800; // Grey is the between Black and White
const int MAX_GRAY = 700; // The Max Value Towards White
const int MIN_GRAY = 600; // The Min Value Towards White
const int MIN_WHITE = 500; // Min White
const int MAX_WHITE = 400; // Max White it can be lower but that's what the sensor mostly reaches

int lineCount = 0; // Counts lines at the start of the race for it to grab the object

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▀ █▀▀ ▀█▀ █░█ █▀█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▄█ ██▄ ░█░ █▄█ █▀▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void setup() {
  strip.begin(); // Initialize neopixels
  Serial.begin(9600); // Start serial monitoring on 9600 for debugging

//--------SETUP THE PIN_MODES---------//
  pinMode(LF, OUTPUT);  // Specify the LeftForward motor to be Output
  pinMode(LB, OUTPUT);  // Specify the LeftBackward motor to be Output
  pinMode(RF, OUTPUT);  // Specify the RightForward motor to be Output
  pinMode(RB, OUTPUT);  // Specify the RightBackward motor to be Output

  pinMode(gripperPin, OUTPUT);  // Specify the gripperpin to be Output
  pinMode(gripperPin, LOW);

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

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░░ █▀█ █▀█ █▀█   █▀▀ █░█ █▄░█ █▀▀ ▀█▀ █ █▀█ █▄░█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▄▄ █▄█ █▄█ █▀▀   █▀░ █▄█ █░▀█ █▄▄ ░█░ █ █▄█ █░▀█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void loop() {

  //Start trigger waits for the flag to be lift up
  if (!startTrigger)
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
    setMotors(140, 0, 164, 0);
    delay(1350);
  } 
  
  
    bool lineScanInProgress = false; // Flag to indicate if line scanning is in progress
    unsigned long currentMillis = millis(); // Get the current time
    scanBlackBox();
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
          if (!startRace)
          {
             scanBlackBox();
            while (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK )
            {
              scanBlackBox_START();

              if (startRace)
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
            setMotors(140, 0, 164, 0);
            pulseIn(2,HIGH,400UL); // Pin , Pulse , Interval
            pulseIn(3,HIGH,400UL);
            pulseIn(2,HIGH,400UL);
            pulseIn(3,HIGH,400UL);
            pulseIn(2,HIGH,400UL);
            pulseIn(3,HIGH,400UL);
            delay(50);
            fullScan();
        }
        if (lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK)
        {
              scanBlackBox_END();
        }
    }
/*
TODO: 
-Optimize the Avoid object
-explain the code 
-incease the labels for sections 
*/
}


//<<<<<<<<<<<<<<<<<<<<<███████╗██╗<<<██╗███╗<<██╗<█████╗<████████╗██╗>█████╗>███╗>>██╗>██████╗>>>>>>>>>>>>>>>>>>>>>//
//<<<<<<<<<<<<<<<<<<<<<██╔════╝██║<<<██║████╗<██║██╔══██╗╚══██╔══╝██║██╔══██╗████╗>██║██╔════╝>>>>>>>>>>>>>>>>>>>>>//
//<<<<<<<<<<<<<<<<<<<<<█████╗<<██║<<<██║██╔██╗██║██║<<╚═╝<>>██║>>>██║██║>>██║██╔██╗██║╚█████╗>>>>>>>>>>>>>>>>>>>>>>//
//<<<<<<<<<<<<<<<<<<<<<██╔══╝<<██║<<<██║██║╚████║██║<<██╗<>>██║>>>██║██║>>██║██║╚████║>╚═══██╗>>>>>>>>>>>>>>>>>>>>>//
//<<<<<<<<<<<<<<<<<<<<<██║<<<<<╚██████╔╝██║<╚███║╚█████╔╝<>>██║>>>██║╚█████╔╝██║>╚███║██████╔╝>>>>>>>>>>>>>>>>>>>>>//
//<<<<<<<<<<<<<<<<<<<<<╚═╝<<<<<<╚═════╝<╚═╝<<╚══╝<╚════╝<<>>╚═╝>>>╚═╝>╚════╝>╚═╝>>╚══╝╚═════╝>>>>>>>>>>>>>>>>>>>>>>//

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▀▄▀█ █▀█ ▀█▀ █▀█ █▀█ █▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░▀░█ █▄█ ░█░ █▄█ █▀▄ ▄█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░░ █ █▄░█ █▀▀   █▀ █▀▀ █▄░█ █▀ █▀█ █▀█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▄▄ █ █░▀█ ██▄   ▄█ ██▄ █░▀█ ▄█ █▄█ █▀▄░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void defaultLineSensor() {
  // Read reflection sensor values
  fullScan();

  if (previoustimeLS_1 < millis())
  {
  for (int i = 0; i < numberOfSensors; i++) {
      if (lineValues[i] > maxSensorValue) {
        maxSensorValue = lineValues[i];
      }
    } 
    previoustimeLS_1 = 30UL + millis();
  }

  
 // Use thresholds to determine the behavior Slowestd on the maximum sensor value
  if (maxSensorValue >= MAX_BLACK) {

    if (lineValues[3] >= MIN_BLACK || lineValues[4] >= MIN_BLACK )
    {
      driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed); //We start with slowest and then modify the speed to a decent speed
      Serial.println("forward");
    }
    else if ( lineValues[2] >= MAX_BLACK)
    {
      driveRight(leftSlowSpeed + speedTurns + additionalSpeed,backwardsSpeedRight);
      Serial.println("right");
    }
    else if (lineValues[5] >= MAX_BLACK)
    {
      driveLeft(backwardsSpeedLeft,rightSlowSpeed + speedTurns + additionalSpeed);
      Serial.println("left");
    }
    else if (lineValues[1] >= MIN_BLACK)
    {
      driveRight(leftSlowSpeed + speedSharpT + additionalSpeed,backwardsSpeedRight);
      Serial.println("sharp right");
    }
    else if (lineValues[6] >= MIN_BLACK)
    {
      driveLeft(backwardsSpeedLeft,rightSlowSpeed + speedSharpT + additionalSpeed);
      Serial.println("sharp left");
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
       
        if (!state) {
            Serial.print("dead");
            Serial.print(" ");
          
          for (int i = 0; i < 20; i++)
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

        if (!state) {

          driveBackward(leftSlowSpeed + speedOneWay,rightSlowSpeed + speedOneWay);
          delay(110);
          Serial.print("dead");
          Serial.print(" ");
          servo(gripOpen);
          endRace = true;
        }
  }

  if (endRace)
    {
      driveBackward(leftSlowSpeed + speedOneWay + additionalSpeed,rightSlowSpeed + speedOneWay + additionalSpeed);
      delay(1350);
    }
    
    while (endRace)
    {
      endLights();
      driveStop();
    }  
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░█░█ █░░ ▀█▀ █▀█ ▄▀█ █▀ █▀█ █▄░█ █ █▀▀   █▀ █▀▀ █▄░█ █▀ █▀█ █▀█░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░█▄█ █▄▄ ░█░ █▀▄ █▀█ ▄█ █▄█ █░▀█ █ █▄▄   ▄█ ██▄ █░▀█ ▄█ █▄█ █▀▄░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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

          Serial.println("left");
          Serial.println(" ");
          driveLeft(backwardsSpeedLeft, rightSlowSpeed + speedTurns + additionalSpeed);
          delay(700);

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(600); // 1000

          Serial.println("right");
          Serial.println(" ");
          driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
          delay(700); //850

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(800);

          Serial.println("right");
          Serial.println(" ");
          driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
          delay(700); //800

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed,rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(600);
          
          Serial.println("left");
          Serial.println(" ");
          driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
          delay(100);
          
        defaultLineSensor();
      }
    else
      {
        Serial.print(distance);
        Serial.println(" cm");
        defaultLineSensor();
      }
       timer = millis() + 100;
    }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▀▀ █▀█ █ █▀█ █▀█ █▀▀ █▀█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▄█ █▀▄ █ █▀▀ █▀▀ ██▄ █▀▄░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▄░█ █▀▀ █▀█ █▀█ █ ▀▄▀ █▀▀ █░░ █▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░▀█ ██▄ █▄█ █▀▀ █ █░█ ██▄ █▄▄ ▄█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//  Function to set the color of a single neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red,green,blue));
  strip.show(); // Set neopixel on off
}

void startLights() {  // Set the color of the neopixels by pixel number and RGB value
  // Pixel number, Red, Green, Blue
  setPixelColor(0, 168, 7, 7); //left back
  setPixelColor(1, 168, 7, 7); //right back
  setPixelColor(2, 168, 7, 7); //right front
  setPixelColor(3, 168, 7, 7); //left front
}

void endLights() {  // Set the color of the neopixels by pixel number and RGB value
  // Pixel number, Red, Green, Blue
  setPixelColor(0, 153, 12, 54); //left back
  setPixelColor(1, 115, 12, 153); //right back
  setPixelColor(2, 153, 97, 12); //right front
  setPixelColor(3, 12, 148, 134); //left front
}
void leftLights() {  // Set the color of the neopixels by pixel number and RGB value
  // Pixel number, Red, Green, Blue
  setPixelColor(0, 134, 29, 153); //left back
  setPixelColor(1, 222, 222, 0); //right back
  setPixelColor(2, 222, 222, 0); //right front
  setPixelColor(3, 134, 29, 153); //left front
}
void rightLights() {  // Set the color of the neopixels by pixel number and RGB value
  // Pixel number, Red, Green, Blue
  setPixelColor(0, 222, 222, 0); //left back
  setPixelColor(1, 29, 114, 163); //right back
  setPixelColor(2, 29, 114, 163); //right front
  setPixelColor(3, 222, 222, 0); //left front
}
void neutralLights() {  // Set the color of the neopixels by pixel number and RGB value
  // Pixel number, Red, Green, Blue
  setPixelColor(0, 222, 222, 0); //left back
  setPixelColor(1, 222, 222, 0); //right back
  setPixelColor(2, 222, 222, 0); //right front
  setPixelColor(3, 222, 222, 0); //left front
}