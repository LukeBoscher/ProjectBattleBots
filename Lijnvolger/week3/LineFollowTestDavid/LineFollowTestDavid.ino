#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NEO_PIXEL_PIN 8 //Attached to digital pin 8
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int MOTOR_A1 = 10;  // Left motor pin 1
const int MOTOR_A2 = 5;   // Left motor pin 2
const int MOTOR_B1 = 6;   // Right motor pin 1
const int MOTOR_B2 = 9;   // Right motor pin 2

const int MOTOR_SENSOR1 = A4;
const int MOTOR_SENSOR2 = A5;


const int TRANSMIT_PIN = 11; 

const int MAX_SPEED = 240;
const int TURN_SPEED = 150;

const int NUM_SENSORS = 8;
const int SENSOR_PINS[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];

// Store last known direction: -1 (left), 1 (right), 0 (center)
int LAST_DIRECTION = 0;

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(MOTOR_SENSOR1, INPUT);
  pinMode(MOTOR_SENSOR2, INPUT);
  pinMode(TRANSMIT_PIN, OUTPUT);

  pixels.begin();
  Serial.begin(9600);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  Serial.println("Beweeg de robot over de lijn voor kalibratie...");
  stop();  
}

void loop() {

  int sensorReadings[NUM_SENSORS];
  bool BLACK_LINE_DETECTED = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
      sensorReadings[i] = analogRead(SENSOR_PINS[i]);

      minValues[i] = min(sensorReadings[i], minValues[i]);
      maxValues[i] = max(sensorReadings[i], maxValues[i]);
  }

  int AVG_VALUE = 0;
  for (int j = 0; j < NUM_SENSORS; j++) {
      AVG_VALUE += (minValues[j] + maxValues[j]) / 2;
  }
  AVG_VALUE /= NUM_SENSORS;
  
  int DEADZONE_LOW = AVG_VALUE - 50;
  int DEADZONE_HIGH = AVG_VALUE + 50;

  // Detect if any sensor sees black
  for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorReadings[i] <= DEADZONE_LOW) { 
          BLACK_LINE_DETECTED = true;
          break;
      }
  }

  // Line Following Logic
  if (sensorReadings[4] >= DEADZONE_HIGH && sensorReadings[5] >= DEADZONE_HIGH) {
      drive(255, 255);  // Move forward
      LAST_DIRECTION = 0;  // Reset direction when moving straight
  }
  else if (sensorReadings[5] >= DEADZONE_HIGH && sensorReadings[6] >= DEADZONE_HIGH) {
      drive(255, 165);  // Slight right
      LAST_DIRECTION = -1;  // Remember last seen black was on the left
      rightSignal();
  } 
  else if (sensorReadings[6] >= DEADZONE_HIGH && sensorReadings[7] >= DEADZONE_HIGH) {
      drive(255, 5);   // More right
      LAST_DIRECTION = -1;
      rightSignal();
  }  
  else if (sensorReadings[7] >= DEADZONE_HIGH && sensorReadings[7] >= DEADZONE_HIGH) {
    drive(255, 55);   // sharp right
    LAST_DIRECTION = -1;
    rightSignal();
  }  
  else if (sensorReadings[3] >= DEADZONE_HIGH && sensorReadings[4] >= DEADZONE_HIGH) {
      drive(165, 255);  // Slight left
      LAST_DIRECTION = 1;  // Remember last seen black was on the right
      leftSignal();
  }  
  else if (sensorReadings[2] >= DEADZONE_HIGH && sensorReadings[3] >= DEADZONE_HIGH) {
      drive(5, 255);   // More left
      LAST_DIRECTION = 1;
      leftSignal();
  }  
  else if (sensorReadings[1] >= DEADZONE_HIGH && sensorReadings[2] >= DEADZONE_HIGH) {
      drive(55, 255);   // Sharp left
      LAST_DIRECTION = 1;
      leftSignal();
  }  
  else {
      // If line is lost, steer towards last known direction
      if (LAST_DIRECTION == -1) {
          drive(255, 0);  // Turn left to search
      } 
      else if (LAST_DIRECTION == 1) {
          drive(0, 255);  // Turn right to search
      } 
      else {
          drive(0, 0);  // Stop if no memory
      } 
      alarm();
  }

  delay(50);  

}

void drive(int SPEED_LEFT, int SPEED_RIGHT) {
  SPEED_LEFT = constrain(SPEED_LEFT, 0, MAX_SPEED);
  SPEED_RIGHT = constrain(SPEED_RIGHT, 0, MAX_SPEED);

  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, SPEED_LEFT);
  analogWrite(MOTOR_B1, SPEED_RIGHT);
  analogWrite(MOTOR_B2, 0);
}

void stop() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void turnLeft() {
  analogWrite(MOTOR_A1, TURN_SPEED);  
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, TURN_SPEED);  
  analogWrite(MOTOR_B2, 0);
}

void turnRight() {
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, TURN_SPEED);
  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, TURN_SPEED);
}

void moveForward() {  
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, MAX_SPEED);
  analogWrite(MOTOR_B1, MAX_SPEED);  
  analogWrite(MOTOR_B2, 0);
}

void moveBackwards() {
  analogWrite(MOTOR_A1, MAX_SPEED);  
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, MAX_SPEED);
}

void slowTurnRight() {
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 125);  
  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 125);
}

void slowTurnLeft() {
  analogWrite(MOTOR_A1, 150);  
  analogWrite(MOTOR_A2, 0);  
  analogWrite(MOTOR_B1, 150);  
  analogWrite(MOTOR_B2, 0);
}

void brakeLight() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 150, 0)); //Set left rear color to orange (G,R,B)
  pixels.setPixelColor(1, pixels.Color(0, 150, 0)); //Set right rear color to orange (G,R,B)
  pixels.show();   // Send the updated pixel colors to the hardware.
}

void rightSignal() {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(1, pixels.Color(70, 255, 0)); //Set right rear color to orange (G,R,B)
    pixels.setPixelColor(2, pixels.Color(70, 255, 0)); //Set right front color to orange (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.

}

void leftSignal() {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(70, 255, 0)); //Set left rear color to orange (G,R,B)
    pixels.setPixelColor(3, pixels.Color(70, 255, 0)); //Set left front color to orange (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.


}

void alarm() {
  pixels.clear(); // Set all pixel colors to 'off'

  // Set all LEDs to red
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void lightShow() 
{
    for (int i = 0; i < 5; i++) { // Repeat the effect 5 times
        // Rainbow cycle effect
        for (int j = 0; j < 256; j += 5) 
        {
            for (int k = 0; k < NUM_PIXELS; k++) {
                pixels.setPixelColor(k, pixels.Color(j, 255 - j, (j * 2) % 255));
            }
            pixels.show();
            delay(50);
        }

        // Strobe effect (all pixels flash white)
        for (int j = 0; j < 5; j++) {
            pixels.fill(pixels.Color(255, 255, 255)); // White
            pixels.show();
            delay(100);
            pixels.clear();
            pixels.show();
            delay(100);
        }

        // Chasing lights effect
        for (int j = 0; j < NUM_PIXELS; j++) 
        {
            pixels.clear();
            pixels.setPixelColor(j, pixels.Color(0, 255, 0)); // Green
            pixels.show();
            delay(200);
        }

        // Random colors flashing
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < NUM_PIXELS; k++) 
            {
                pixels.setPixelColor(k, pixels.Color(random(255), random(255), random(255)));
            }
            pixels.show();
            delay(200);
        }
    }

    pixels.clear();
    pixels.show();
}


