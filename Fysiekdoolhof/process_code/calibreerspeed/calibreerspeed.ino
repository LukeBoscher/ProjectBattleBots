const int motor_L1 = 6;                                                               //achteruit drive(-241, -254)
const int motor_L2 = 11;                                                              //vooruit drive(214, 255)
const int motor_R1 = 10;
const int motor_R2 = 5;
const int buttonOn = 7;
const int buttonOff = 4;
int buttonState = 0;
int buttonState2 = 0;
const int echo = 9;
const int trig = 3;
float duration, distance;
const int lightsensor1= A0;
const int lightsensor2= A1;
const int lightsensor3= A2;
const int lightsensor4= A3;
const int lightsensor5= A4;
const int lightsensor6= A5;
const int lightsensor7= A6;
const int lightsensor8= A7;
#define MAX_DISTANCE 200  // Maximale meetafstand in cm
#define NUM_MEASUREMENTS 5 // Aantal metingen voor filtering
const int motorsensor1 = 4;
const int motorsensor2 = 8;
int cws1 = 0, cws2 = 0;  // Telling voor wielsensoren
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval van 1 seconde
const int fullspeedLinksVooruit = 214;
const int fullspeedLinksAchteruit = -241.5;

void setup() {
  pinMode(motor_L1, OUTPUT);
  pinMode(motor_L2, OUTPUT);
  pinMode(motor_R1, OUTPUT);
  pinMode(motor_R2, OUTPUT);
  pinMode(buttonOn, INPUT);
  pinMode(buttonOff, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(motorsensor1, INPUT);
  pinMode(motorsensor2, INPUT);

  Serial.begin(9600);
  stop();  // Ensure motors are stopped at startup
  drive(255,255);
}

// Functie om de motoren vooruit te laten bewegen
void drive(int speedLeft, int speedRight) {
  if (speedLeft >= 0) {
    analogWrite(motor_L1, 0);
    analogWrite(motor_L2, speedLeft);
  } else {
    analogWrite(motor_L1, -speedLeft);  // Negatieve waarde om achteruit te draaien
    analogWrite(motor_L2, 0);
  }

  if (speedRight >= 0) {
    analogWrite(motor_R1, 0);
    analogWrite(motor_R2, speedRight);
  } else {
    analogWrite(motor_R1, -speedRight); // Negatieve waarde om achteruit te draaien
    analogWrite(motor_R2, 0);
  }
}

// Functie om de motoren te stoppen
void stop() {
  analogWrite(motor_L1, 0);
  analogWrite(motor_L2, 0);
  analogWrite(motor_R1, 0);
  analogWrite(motor_R2, 0);
}

// Functie om de afstand te meten met behulp van de ultrasonic sensor
void measureDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = (duration * 0.0343) / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}

// Functie voor het uitvoeren van beweging met sensoren
void performActionWithSensor(void (*actionFunc)(int, int), int speedLeft, int speedRight, int durationMs) {
  unsigned long startTime = millis();
  actionFunc(speedLeft, speedRight);  // Start bewegen
  while (millis() - startTime < durationMs) {
    measureDistance();  // Blijf afstand meten
  }
  stop();  // Stop na de actie
}

// Functie om het aantal pulsen van motorsensoren te lezen
void readMotorSensors() {
  cws1 = pulseIn(motorsensor1, HIGH);  // Lees het aantal pulsen voor motorsensor 1
  cws2 = pulseIn(motorsensor2, HIGH);  // Lees het aantal pulsen voor motorsensor 2
}

int speedLeft = -255;

int getPulseDifference() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;  // Reset de tijd bij het interval
    readMotorSensors();  // Lees de pulsen van de motorsensoren

    // Bereken het verschil in pulsen
    int pulseDifference = cws1 - cws2;
    pulseDifference = (pulseDifference / 1200);

    // Toon het verschil in pulsen in de seriële monitor
    Serial.print("Pulse Difference: ");
    Serial.println(pulseDifference);
    return pulseDifference;

  }
}

void loop() {
  drive(speedLeft, -254);
  buttonState = digitalRead(buttonOn);
  buttonState2 = digitalRead(buttonOff);



  while(true) {
    int pulseDifference = getPulseDifference();
    drive(speedLeft, -254);
    delay(1000);
    pulseDifference = getPulseDifference();
    if (pulseDifference > 0) {
      speedLeft--;
      pulseDifference = getPulseDifference();
      Serial.println(speedLeft);
    } 
    else if (pulseDifference < 0) {  // Correcte syntax voor else if                                                                    deze code bekijken!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      speedLeft++;
      pulseDifference = getPulseDifference();
      Serial.println(speedLeft);
    } 
    else if(pulseDifference == 0) {
      speedLeft = speedLeft;
    }
}

  // Als de uit-knop wordt ingedrukt, stop de motoren
  if (buttonState2 == LOW) {
    stop();
  }

}
