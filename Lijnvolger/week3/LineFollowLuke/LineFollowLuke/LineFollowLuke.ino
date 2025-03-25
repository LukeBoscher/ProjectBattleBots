#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NeoLED 13
#define NUM_PIXELS 4
#define DELAY_TIME 500

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

const int motor_L1 = 6;
const int motor_L2 = 11;
const int motor_R1 = 10;
const int motor_R2 = 5;
const int buttonOn = 7;
const int buttonOff = 4;
const int echo = 9;
const int trig = 3;
float duration, distance;
#define MAX_DISTANCE 200  
#define NUM_MEASUREMENTS 5 
const int motorsensor1 = A4;
const int motorsensor2 = A5;
int cws1 = 0, cws2 = 0;  
unsigned long previousMillis = 0;
const long interval = 1000;  
const int servoPin = 12;

const int numSensors = 8;
const int sensorPins[numSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};

int minValues[numSensors];
int maxValues[numSensors];

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
  pinMode(servoPin, OUTPUT);

  Serial.begin(9600);

  for (int i = 0; i < numSensors; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  Serial.println("Beweeg de robot over de lijn voor kalibratie...");
  strip.begin();
  strip.show();
  stop();  
}

// Motor control function
void drive(int speedLeft, int speedRight) {
  speedLeft = constrain(speedLeft, 0, 255);
  speedRight = constrain(speedRight, 0, 255);

  analogWrite(motor_L1, speedLeft > 0 ? 0 : -speedLeft);
  analogWrite(motor_L2, speedLeft > 0 ? speedLeft : 0);

  analogWrite(motor_R1, speedRight > 0 ? 0 : -speedRight);
  analogWrite(motor_R2, speedRight > 0 ? speedRight : 0);
}

// Stop motors
void stop() {
  analogWrite(motor_L1, 0);
  analogWrite(motor_L2, 0);
  analogWrite(motor_R1, 0);
  analogWrite(motor_R2, 0);
}

// Measure distance
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
}

// Read wheel sensor pulses
void readMotorSensors() {
  cws1 = pulseIn(motorsensor1, HIGH);
  cws2 = pulseIn(motorsensor2, HIGH);
}

int getPulseDifference() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    readMotorSensors();

    int pulseDifference = cws1 - cws2;
    pulseDifference /= 1200;

    Serial.print("Pulse Difference: ");
    Serial.println(pulseDifference);
    return pulseDifference;
  }
  return 0;
}

// Store last known direction: -1 (left), 1 (right), 0 (center)
int lastDirection = 0;

void loop() {
  int sensorReadings[numSensors];
  bool blackDetected = false;

  for (int i = 0; i < numSensors; i++) {
      sensorReadings[i] = analogRead(sensorPins[i]);

      minValues[i] = min(sensorReadings[i], minValues[i]);
      maxValues[i] = max(sensorReadings[i], maxValues[i]);
  }

  int result = 0;
  for (int j = 0; j < numSensors; j++) {
      result += (minValues[j] + maxValues[j]) / 2;
  }
  result /= numSensors;
  
  int deadzonelow = result - 50;
  int deadzonehigh = result + 50;

  // Detect if any sensor sees black
  for (int i = 0; i < numSensors; i++) {
      if (sensorReadings[i] <= deadzonelow) { 
          blackDetected = true;
          break;
      }
  }

  // Line Following Logic
  if (sensorReadings[4] >= deadzonehigh && sensorReadings[5] >= deadzonehigh) {
      drive(214, 255);  // Move forward
      lastDirection = 0;  // Reset direction when moving straight
  }
  else if (sensorReadings[5] >= deadzonehigh && sensorReadings[6] >= deadzonehigh) {
      drive(255, 170);  // Slight left
      lastDirection = -1;  // Remember last seen black was on the left
  } 
  else if (sensorReadings[6] >= deadzonehigh && sensorReadings[7] >= deadzonehigh) {
      drive(255, 10);   // More left
      lastDirection = -1;
  }  
  else if (sensorReadings[3] >= deadzonehigh && sensorReadings[4] >= deadzonehigh) {
      drive(170, 255);  // Slight right
      lastDirection = 1;  // Remember last seen black was on the right
  }  
  else if (sensorReadings[2] >= deadzonehigh && sensorReadings[3] >= deadzonehigh) {
      drive(60, 255);   // More right
      lastDirection = 1;
  }  
  else if (sensorReadings[1] >= deadzonehigh && sensorReadings[2] >= deadzonehigh) {
      drive(10, 255);   // Sharp right
      lastDirection = 1;
  }  
  else {
      // **If line is lost, steer towards last known direction**
      if (lastDirection == -1) {
          drive(255, 0);  // Turn left to search
      } 
      else if (lastDirection == 1) {
          drive(0, 255);  // Turn right to search
      } 
      else {
          drive(0, 0);  // Stop if no memory
      }
  }

  delay(50);  
}

