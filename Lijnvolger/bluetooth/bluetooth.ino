//transmit = 11
const int TRANSMIT_PIN= 11;


char data = 0;

void setup() {  
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(TRANSMIT_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
    ;
  {

    data = Serial.read();
    Serial.print(data);
    Serial.print("yes");
    Serial.println();
    delay(1000);

  }
}
