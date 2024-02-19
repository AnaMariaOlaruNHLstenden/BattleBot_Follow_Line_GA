
int ledRed = 11; 
int brightness = 0;

void setup() { 

  pinMode(ledRed, OUTPUT); 

} 

void loop() { 

  for (brightness = 0; brightness <= 255; brightness += 15) { 

    analogWrite(ledRed, brightness); 

    delay(20); 

  } 

} 