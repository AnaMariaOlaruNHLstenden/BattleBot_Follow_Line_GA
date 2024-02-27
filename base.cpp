#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
// ^ Call libraries

#define PIN 8 // Neopixel pin NI(Input)
#define NUMPIXELS 4 // There are 4 neopixels on the board

const int echoPin = 4; // Echo sensor echo pin
const int triggerPin = 2; // Echo sensor trigger pin

const int gripperPin = 12; // Gripper pin GR

const int motorLeftFWD = 11; // Motor pin A1
const int motorLeftBWD = 10; // Motor pin A2
const int motorRightFWD = 9; // Motor pin B1
const int motorRightBWD = 6; // Motor pin B2

const int motorPulseLeft = 5; // Motor pin R1
const int motorPulseRight = 3; // Motor pin R2

const int lineSensor1 = A0; // Linesensor pin
const int lineSensor2 = A1; // Linesensor pin
const int lineSensor3 = A2; // Linesensor pin
const int lineSensor4 = A3; // Linesensor pin
const int lineSensor5 = A4; // Linesensor pin
const int lineSensor6 = A5; // Linesensor pin
const int lineSensor7 = A6; // Linesensor pin
const int lineSensor8 = A7; // Linesensor pin

void setup() {
}

void loop() {

}