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
void reconsiderLifeChoices();
void distanceSensor();
void distanceReader();
void servo(int pulse);
void musicPinkMarioRickRoll();

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
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▀▄▀█ █░█ █▀ █ █▀▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░▀░█ █▄█ ▄█ █ █▄▄░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//Pin for Speaker
const int buzzer = 7;

//Defining the notes for speaker
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░█ ▄▀█ █▀█ █ ▄▀█ █▄▄ █░░ █▀▀ █▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▀▄▀ █▀█ █▀▄ █ █▀█ █▄█ █▄▄ ██▄ ▄█░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

//-------------------------------MUSIC------------------------------//
// change this to make the song slower or faster
int tempo = 150; //114

int melody[] = {

  // Never Gonna Give You Up - Mario Astley feat Pink Panther
  // Score available at https://musescore.com/chlorondria_5/never-gonna-give-you-up_alto-sax
  // Arranged by Chlorondria

  NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_E5,8, REST,8, NOTE_C5,8, NOTE_E5,8, //1
  NOTE_G5,4, REST,4, NOTE_G4,8, REST,4, 
  NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4, // 3
  NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
  NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
  REST,8, NOTE_E5,4,NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
  NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4, // repeats from 3
  NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
  NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
  REST,8, NOTE_E5,4,NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,

  NOTE_D5,2, NOTE_D5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,4, 
  NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8, NOTE_A4,8, NOTE_A4,4,

  REST,-4, NOTE_A4,8, NOTE_B4,8, NOTE_CS5,8, NOTE_D5,8, NOTE_B4,8, //35
  REST,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_E5,-8, NOTE_E5,-8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8, 

   NOTE_E5,4, NOTE_D5,2, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16, //40
  NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_A5,4, NOTE_CS5,8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8,  
  NOTE_E5,4, NOTE_D5,2, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
   
  NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16, //45
  NOTE_A5,4, NOTE_CS5,8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8,  
  NOTE_E5,4, NOTE_D5,2, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16, //45
  
  NOTE_A5,4, NOTE_CS5,8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8, 

  NOTE_E5,4, NOTE_D5,2, REST,4,
  REST,2, REST,2, REST,2, NOTE_DS4,8, 
  NOTE_E4,-4, REST,8, NOTE_FS4,8, NOTE_G4,-4, REST,8, NOTE_DS4,8,
  NOTE_E4,-8, NOTE_FS4,8,  NOTE_G4,-8, NOTE_C5,8, NOTE_B4,-8, NOTE_E4,8, NOTE_G4,-8, NOTE_B4,8,   
  NOTE_AS4,2, NOTE_A4,-16, NOTE_G4,-16, NOTE_E4,-16, NOTE_D4,-16, 
  NOTE_E4,2, REST,4, REST,8, NOTE_DS4,4,

  NOTE_E4,-4, REST,8, NOTE_FS4,8, NOTE_G4,-4, REST,8, NOTE_DS4,8,
  NOTE_E4,-8, NOTE_FS4,8,  NOTE_G4,-8, NOTE_C5,8, NOTE_B4,-8, NOTE_G4,8, NOTE_B4,-8, NOTE_E5,8,
  NOTE_DS5,1,   
  NOTE_D5,2, REST,4, REST,8, NOTE_DS4,8, 
  NOTE_E4,-4, REST,8, NOTE_FS4,8, NOTE_G4,-4, REST,8, NOTE_DS4,8,
  NOTE_E4,-8, NOTE_FS4,8,  NOTE_G4,-8, NOTE_C5,8, NOTE_B4,-8, NOTE_E4,8, NOTE_G4,-8, NOTE_B4,8,   
  
  NOTE_AS4,2, NOTE_A4,-16, NOTE_G4,-16, NOTE_E4,-16, NOTE_D4,-16, 
  NOTE_E4,-4, REST,4,
  REST,4, NOTE_E5,-8, NOTE_D5,8, NOTE_B4,-8, NOTE_A4,8, NOTE_G4,-8, NOTE_E4,-8,
  NOTE_AS4,16, NOTE_A4,-8, NOTE_AS4,16, NOTE_A4,-8, NOTE_AS4,16, NOTE_A4,-8, NOTE_AS4,16, NOTE_A4,-8,   
  NOTE_G4,-16, NOTE_E4,-16, NOTE_D4,-16, NOTE_E4,16, NOTE_E4,16, NOTE_E4,2,
};

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

//-------BOOL-------//
 bool startTrigger = false; // Flag Verification
 bool startRace = false; // Start Verification
 bool endRace = false; // End Verification

//-------ULTRASONIC-------//
const int maxDistance = 18; // 20 cm maximum distance to leave room for error
const int startDistance = 25; // 15 cm maximum distance from the flag
float distance, duration; // declared 2 floats in 1 line for organization purposes
const int echoInterval = 100; // Time between seeing the object and calculating the distance

//--------GRIPPER-------//
int gripOpen = 1950; // pulse length servo open
int gripClosed = 1100; // pulse length servo closed
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
    for (int i = 0; i < 50; i++)
    {
    servo(gripOpen);
    delayMicroseconds(1000);
    servo(gripOpen);
    }
    while (distance < startDistance)
    {
      static unsigned long timer;
      if (millis() > timer) 
      {
      driveStop();
      Serial.println(distance);
      distanceReader();
        
        if (distance > startDistance)
        {
          break;
        }
            timer = millis() + 200;
      }
    } 
    startTrigger = true;
    Serial.println("left while loop");
    setMotors(255, 0, 255, 0);
    delay(60);
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
    
    defaultLineSensor(); //Reading the line
    distanceSensor(); //Detecting the object and avoiding it
    
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
  setMotors(0, leftSpeed, 0, rightSpeed);
}

void driveRight(int leftSpeed, int rightSpeed) {
  rightLights();
  setMotors(leftSpeed, 0, 0, rightSpeed);
}

void driveLeft(int leftSpeed, int rightSpeed) {
  leftLights();
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

  static unsigned long previousTime;

  if ((millis() - previousTime) >= 100UL)
  {
  for (int i = 0; i < numberOfSensors; i++) {
      if (lineValues[3] > maxSensorValue && lineValues[4] > maxSensorValue)
      {
        maxSensorValue = lineValues[3];
      }
    } 
     previousTime = millis();
  }

    // Uses thresholds to determine the behavior on the maximum sensor value
    if (maxSensorValue >= MAX_BLACK) {

      if (lineValues[3] >= MIN_BLACK || lineValues[4] >= MIN_BLACK )
      {
        //We start with slowest and then modify the speed to a decent speed
        driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed); 
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

void scanBlackBox() //Reading the black box
{
   for (int i = 0; i < 2; i++) 
  {
    lineValues[0] = analogRead(lineSensor[0]); 
    lineValues[7] = analogRead(lineSensor[7]); 
  }  
}

void fullScan() //Reads with all sensors
{
  for (int i = 0; i < numberOfSensors; i++) 
  {
    lineValues[i] = analogRead(lineSensor[i]);
  } 
}

//This is the start where after it counted the lines, it checks one time and then it's hardcoded to go left. 
//Modifying this leads to the bot being unable to see the line again.
void scanBlackBox_START()
{

  if (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK )
  {
    Serial.print("I see le box");
      Serial.print(" ");

    for (int i = 0; i < 50; i++)
    {
      servo(gripClosed);
      delayMicroseconds(1000);
      servo(gripClosed);
    
    startRace = true;
    }
  }
  driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
  delay(1345);
  driveForward(leftSlowSpeed + startSpeed,rightSlowSpeed + startSpeed);
  defaultLineSensor();
}

//END
//This procedure it's set to make the bot to read the box for last time and open the gripper finishing with a song.
void scanBlackBox_END()
{

  fullScan();

  if(lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK)
  {
    Serial.print("I see le box");
    Serial.print(" ");

    driveBackward(leftSlowSpeed + speedOneWay,rightSlowSpeed + speedOneWay);
    delay(125);
    Serial.print("dead");
    Serial.print(" ");
    servo(gripOpen);

    endRace = true; 
  }

  if (endRace)
    {
      driveBackward(leftSlowSpeed + speedOneWay + additionalSpeed,rightSlowSpeed + speedOneWay + additionalSpeed);
      delay(1350);
      driveStop();
      endLights();
      musicPinkMarioRickRoll();
    }

    while (endRace) //Trapping the system in a while loop for safety of the bot.
    {
      driveStop();
      endLights();
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
    
    if (distance <= maxDistance && lineCount >= 4) //Condition that it counted 4 lines before to avoid conflict with the start code.
      {
        Serial.println("Avoid Object"); //This function makes sure that anything closer than 20CM it will avoid it

          Serial.println("back");
          Serial.println(" ");
          driveBackward(leftSlowSpeed + speedOneWay + additionalSpeed,rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(200);

          Serial.println("left");
          Serial.println(" ");
          driveLeft(backwardsSpeedLeft, rightSlowSpeed + speedTurns + additionalSpeed);
          delay(700);

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(500); // 1000 

          Serial.println("right");
          Serial.println(" ");
          driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
          delay(750); //850

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(750);

          Serial.println("right");
          Serial.println(" ");
          driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
          delay(700); //800

          Serial.println("forward");
          Serial.println(" ");
          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed,rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(500); //600
          
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

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█▀▄▀█ █░█ █▀ █ █▀▀░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█░▀░█ █▄█ ▄█ █ █▄▄░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░//
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<o>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void musicPinkMarioRickRoll()
{
  // Iterate over the notes of the melody.
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzer);
  }
}
