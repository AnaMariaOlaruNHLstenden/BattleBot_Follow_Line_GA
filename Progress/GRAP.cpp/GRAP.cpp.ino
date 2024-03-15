const int GRIP = 12

void setup() {
  
   pinMode(GRIP, OUTPUT);
  digitalWrite(GRIP, LOW);
  grab();

void grab(){
  Serial.println("grab");
  for(int i = 0; i < 15; i++){
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(19000);
  }
}

void ungrab(){
 Serial.println("ungrab");
  for(int i = 0; i < 15; i++){
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1500);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(18500);
  }
}
//GRIP should be a number of pin connected to gripper
 
