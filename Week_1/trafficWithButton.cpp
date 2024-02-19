const int buttonPin1 = 2; 
const int buttonPin2 = 3; 
const int buttonPin3 = 4; 
const int ledPin1 = 13; 
const int ledPin2 = 12; 
const int ledPin3 = 11; 
 
int buttonState1 = 0; 
int buttonState2 = 0; 
int buttonState3 = 0; 
 
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
 
  if (buttonState1 == LOW){ 
    digitalWrite(ledPin1, LOW);    
    digitalWrite(ledPin2, HIGH);    
    digitalWrite(ledPin3, HIGH);  
    delay(3000);     
 
    digitalWrite(ledPin1, HIGH);     
    digitalWrite(ledPin2, HIGH);    
    digitalWrite(ledPin3, LOW);  
    delay(4000);     
 
    digitalWrite(ledPin1, HIGH);     
    digitalWrite(ledPin2, LOW);    
    digitalWrite(ledPin3, HIGH);  
    delay(1000);   
  } else { 
    digitalWrite(ledPin1, HIGH); 
    digitalWrite(ledPin2, HIGH); 
    digitalWrite(ledPin3, HIGH); 
  } 
} 