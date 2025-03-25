void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(1000);                     
  digitalWrite(LED_BUILTIN, LOW);   
  delay(1000);                     
}
