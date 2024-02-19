const int buttonPin1 = 2; 
const int buttonPin2 = 3; 
const int buttonPin3 = 4; 
const int ledPin1 = 13; 
const int ledPin2 = 12; 
const int ledPin3 = 11; 
 
int buttonState1 = 0; 
int buttonState2 = 0; 
int buttonState3 = 0; 
 
unsigned long timeOne = 0; 
bool state; 
 
void setup() { 
  pinMode(ledPin1, OUTPUT); 
  pinMode(buttonPin1, INPUT); 
 
  pinMode(ledPin2, OUTPUT); 
  pinMode(buttonPin2, INPUT); 
 
  pinMode(ledPin3, OUTPUT); 
  pinMode(buttonPin3, INPUT); 
} 
 
void loop(){ 
  buttonState1 = digitalRead(buttonPin1); 
  buttonState2 = digitalRead(buttonPin2); 
  buttonState3 = digitalRead(buttonPin3); 
 
  digitalWrite(ledPin2, HIGH); 
  blink(100); 
 
  if (buttonState1 == LOW || buttonState2 == LOW || buttonState3 == LOW){ 
    digitalWrite(ledPin3, HIGH);  
  } else { 
    digitalWrite(ledPin3, LOW); 
  } 
} 
 
void blink(int INTERVAL){ 
  if (millis() >= timeOne){ 
    timeOne = millis() + INTERVAL; 
 
    state = !state; 
    digitalWrite(ledPin1, state); 
  } 
} 