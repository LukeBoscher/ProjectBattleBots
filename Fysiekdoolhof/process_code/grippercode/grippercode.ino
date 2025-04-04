#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NeoLED 13
#define NUM_PIXELS 4
#define DELAY_TIME 500

//servo
#define SERVO 12
#define GRIPPER_OPEN 1550
#define GRIPPER_CLOSE 1000


Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

//motor pins
#define motor_L1 6
#define motor_L2 11
#define motor_R1 10
#define motor_R2 5

//buttons
const int buttonOn = 7;
const int buttonOff = 4;
int buttonState = 0;
int buttonState2 = 0;

//oogjes pins
#define echo 9
#define trig 3

float duration, distance;
#define MAX_DISTANCE 200  // Maximale meetafstand in cm
#define NUM_MEASUREMENTS 5 // Aantal metingen voor filtering
//motor sensor
#define motorsensor1 A4
#define motorsensor2 A5
int cws1 = 0, cws2 = 0;  // Telling voor wielsensoren
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval van 1 seconde
const int fullspeedLinksVooruit = 214;
const int fullspeedLinksAchteruit = -241.5;
int lichtsensorWaarden[8];  // Array om waarden van 8 sensoren op te slaan
const int lichtsensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Sensor pinnen


void setup() {

  // motor pins
  pinMode(motor_L1, OUTPUT);
  pinMode(motor_L2, OUTPUT);
  pinMode(motor_R1, OUTPUT);
  pinMode(motor_R2, OUTPUT);

  pinMode(motor_L1 ,HIGH);
  pinMode(motor_L2 ,HIGH);
  pinMode(motor_R1 ,HIGH);
  pinMode(motor_R2 ,HIGH);
  // buttonpins
  pinMode(buttonOn, INPUT);
  pinMode(buttonOff, INPUT);

  // supersoon sensor pins
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(motorsensor1, INPUT);
  pinMode(motorsensor2, INPUT);

  // lijnsensorpins
  for (int i=0; i < 8; i++)
  {
    pinMode(lichtsensorPins[i], INPUT);

  }


  //servo
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
  Serial.begin(9600);

  // niti
  strip.begin();
  strip.show();
  stop();  // Ensure motors are stopped at startup
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

// Functie om het aantal pulsen van motorsensoren te lezen
void readMotorSensors() {
  cws1 = pulseIn(motorsensor1, HIGH);  // Lees het aantal pulsen voor motorsensor 1
  cws2 = pulseIn(motorsensor2, HIGH);  // Lees het aantal pulsen voor motorsensor 2
}

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

void moveServo(int pulse) {
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer) {
    if (pulse > 0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }
    digitalWrite(SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20; //20ms interval voor servo
  }
}

// void led1() {
//   static unsigned long timer;
//   static bool state;
//   if (millis() > timer) {
//     if (state) {
//       digitalWrite(LED_1, LOW);
//     } else {
//       digitalWrite(LED_1, HIGH);
//     }
//   }
//   state = !state;
//   timer = millis() + LED_1_INTERVAL;
// }


void readLightSensors() {
    for (int i = 0; i < 8; i++) {
        lichtsensorWaarden[i] = analogRead(lichtsensorPins[i]);  // Lees elke sensor
    }
}

void printLightSensorValues() {
    for (int i = 0; i < 8; i++) {
        Serial.print("Sensor ");
        Serial.print(i + 1);  // i+1 to show 1-based index
        Serial.print(": ");
        Serial.println(lichtsensorWaarden[i]);  // Print sensor value
    }
}


void loop() {
    
    if (buttonState == LOW) {
        unsigned long startTime = millis();  // Start time for the sequence

        while (millis() - startTime < 4500) {  // Run for 4500ms
            unsigned long elapsedTime = millis() - startTime;

            if (elapsedTime < 1000) {
                moveServo(GRIPPER_OPEN);
            } else if (elapsedTime < 1500) {
                drive(214, 255);  // Move forward a small distance
            } else if (elapsedTime < 2500) {
                moveServo(GRIPPER_CLOSE);
            } else if (elapsedTime < 4500) {
                drive(214, 255);  // Move forward for 2 seconds
            } else {
                break;  // Exit loop after 4500ms
            }
        }

        stop();  // Ensure the robot stops after the sequence ends

        #ifdef DEBUG
        Serial.println("Klaar");
        #endif
    }
}



