#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN 8 // Neopixel pin NI(Input)
#define NUMPIXELS 4 // There are 4 neopixels on the board

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800); // Neopixel needed code from library

//  Function to set the color of a single neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red,green,blue));
  strip.show(); // Set neopixel on off
}

void testLights() {
  // Set the color of the neopixels by pixel number and rgb value

  // Pixel number, Green, Red, Blue, Yellow
  setPixelColor(0, 255, 255, 0); //left back
  setPixelColor(1, 255, 255, 0); //right back
  setPixelColor(2, 255, 255, 0); //right front
  setPixelColor(3, 255, 255, 0); //left front
}

void setup() {
  strip.begin(); // Initialize neopixels
  strip.show(); // Set neopixel on off
  Serial.begin(9600); // Start serial monitoring on 9600
  testLights(); // Call the function to set the neopixel lights
}

void loop() {

}