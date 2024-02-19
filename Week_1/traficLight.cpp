
const int L1 = 13; 
const int L2 = 12; 
const int L3 = 11; 

void setup() { 

  pinMode(L1, OUTPUT); 
  pinMode(L2, OUTPUT); 
  pinMode(L3, OUTPUT); 

} 


void loop() { 

  digitalWrite(L1, LOW);   
  digitalWrite(L2, HIGH);   
  digitalWrite(L3, HIGH); 

  delay(3000);

  digitalWrite(L1, HIGH);    
  digitalWrite(L2, HIGH);   
  digitalWrite(L3, LOW); 

  delay(4000);

  digitalWrite(L1, HIGH);    
  digitalWrite(L2, LOW);   
  digitalWrite(L3, HIGH); 

  delay(1000);

} 