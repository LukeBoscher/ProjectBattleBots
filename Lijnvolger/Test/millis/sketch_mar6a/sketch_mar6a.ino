  const unsigned long interval = 1000;
  unsigned long previousTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(millis());

  if (milllis() > 1000)
  {
    Serial.print("it works");

    unsigned long currentTime = millis();

    if(currentTime - previousTime >= interval)
    {
      Serial.print("ihbehud")

      previousTime = currentTime;
    }
  }
}
