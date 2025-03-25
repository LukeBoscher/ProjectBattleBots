#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NEO_PIXEL_PIN 8 //Attached to digital pin 8
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int SERVO_PIN = 3; 

const int MOTOR_A1 = 10;  // Left motor pin 1
const int MOTOR_A2 = 5;   // Left motor pin 2
const int MOTOR_B1 = 6;   // Right motor pin 1
const int MOTOR_B2 = 9;   // Right motor pin 2

const int MAX_SPEED = 255;
const int MID_SPEED = 220;

// Range how the gripper open and closes. 360 degrees
int OPENGRIP_VALUE = 120;
int CLOSEGRIP_VALUE = 50;

unsigned long currentMillis = 0;
unsigned long previousActionMillis = 0;
unsigned long previousServoMillis = 0;
const long SERVO_INTERVAL = 20; // 20 ms delay

const long OPEN_TIME = 0;            // Open grip at start
const long CLOSE_TIME = 1000;        // Close grip after 1s
const long OPEN_AGAIN_TIME = 2000;    // Open grip again after 2s
const long DRIVE_START_TIME = 3000;   // Start driving after 3s
const long GRAB_TIME = 5000;         // Grab cone after 4s 
const long HOLD_TIME = 7000;         // Stop and hold after 6s
const long DROP_TIME = 9000;        // Drop after 8s
const long DRIVE_AGAIN_TIME = 6000;  // Drive again after 5s

boolean isGripClosed = false;

void generatePulse(int angle) {
  int pulseWidth = map(angle, 0, 180, 544, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}

void setup() 
{
  pixels.begin(); // INITIALIZE NeoPixel strip object
  
  // Initialize serial communication:
  Serial.begin(9600);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() 
{ 
  pickUp();
}

void pickUp() {
    currentMillis = millis();
    unsigned long elapsedTime = currentMillis - previousActionMillis;
    
    // Keep servo active
    if(currentMillis - previousServoMillis >= SERVO_INTERVAL) 
    {
      previousServoMillis = currentMillis;
      if(isGripClosed) 
      {
        generatePulse(CLOSEGRIP_VALUE);
      } 
      else 
      {
        generatePulse(OPENGRIP_VALUE);
      }
    }
    
    // Open the grip
    if(elapsedTime >= OPEN_TIME && elapsedTime < CLOSE_TIME && isGripClosed) 
    {
      isGripClosed = false;
    }
    
    // Close it after 1 second
    else if(elapsedTime >= CLOSE_TIME && elapsedTime < OPEN_AGAIN_TIME && !isGripClosed)
     {
      isGripClosed = true;
    }
    
    // Open it again after 1 more second
    else if(elapsedTime >= OPEN_AGAIN_TIME && elapsedTime < DRIVE_START_TIME && isGripClosed) 
    {
      isGripClosed = false;
    }
    
    // Drive forward to the cone
    else if(elapsedTime >= DRIVE_START_TIME && elapsedTime < GRAB_TIME) 
    {
      moveForward();
    }
    
    // Grab the cone
    else if(elapsedTime >= GRAB_TIME && elapsedTime < HOLD_TIME && !isGripClosed)
     {
      stopMotor();
      isGripClosed = true;
    }
    
    // Drive forward again with the cone
    else if(elapsedTime >= DRIVE_AGAIN_TIME && elapsedTime < HOLD_TIME)
     {
      moveForward();
    }
    
    // Stop and hold the cone
    else if(elapsedTime >= HOLD_TIME && elapsedTime < DROP_TIME) 
    {
      stopMotor();
    }
    
    // Drop the cone after the specified time
    else if (elapsedTime >= DROP_TIME) 
    {
      isGripClosed = false;
    }
}

void moveForward() {  
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, MID_SPEED);

  analogWrite(MOTOR_B1, MID_SPEED);  
  analogWrite(MOTOR_B2, 0);
} 

// Function to make robot stop
void stopMotor(){ 
  brakeLight();
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 0);
}

void brakeLight() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 150, 0)); // Set left rear color to orange (G,R,B)
  pixels.setPixelColor(1, pixels.Color(0, 150, 0)); // Set right rear color to orange (G,R,B)
  pixels.show();   // Send the updated pixel colors to the hardware.
}

void rightSignal() {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(1, pixels.Color(70, 255, 0)); // Set right rear color to green (G,R,B)
    pixels.setPixelColor(2, pixels.Color(70, 255, 0)); // Set right front color to green (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.
}

void leftSignal() {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(70, 255, 0)); // Set left rear color to green (G,R,B)
    pixels.setPixelColor(3, pixels.Color(70, 255, 0)); // Set left front color to green (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.
}