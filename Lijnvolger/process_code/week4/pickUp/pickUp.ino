#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NEO_PIXEL_PIN 8 //Attached to digital pin 8
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int SERVO_PIN = 3; 

// Motor A1 is pin 10
const int MOTOR_A1 = 10;
// Motor A2 is pin 5
const int MOTOR_A2 = 5;
// Motor B1 is pin 6
const int MOTOR_B1 = 6;
// Motor B2 is pin 9
const int MOTOR_B2 = 9;

const int MAX_SPEED = 255;

//range how the gripper open and closes. 360 degrees
int OPENGRIP_VALUE = 120;
int CLOSEGRIP_VALUE = 50;

int INTERVAL = 1000;
int PREVIOUSMILLIS = 0;

void setup() 
{
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  // put your setup code here, to run once:
  // initialize serial communication:
  Serial.begin(9600);

  //motor A have output
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);

  //motor B have output
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);

}


void loop() 
{  
  openGrip();
  millisDelay(1000);

  closeGrip();
  millisDelay(1000);

  openGrip();
  millisDelay(1000);

  moveForward();
  millisDelay(2000);

  stopMotor();
  millisDelay(1000);


  closeGrip();
  millisDelay(1000);
  pickUp(2000);

  stopMotor();
}

void generatePulse(int ANGLE){
  int PULSEWIDTH = map(ANGLE, 0, 180, 544, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(PULSEWIDTH);
  digitalWrite(SERVO_PIN, LOW);
}

void openGrip(){
  generatePulse(OPENGRIP_VALUE);
}

void closeGrip()
{
  generatePulse(CLOSEGRIP_VALUE);
}

void pickUp(unsigned long DURATION) 
{
  unsigned long START_TIME = millis(); // Record the start time
  while (millis() - START_TIME < DURATION) {
    moveForward();
  }
}

void millisDelay(unsigned long DURATION) 
{
  unsigned long START_TIME = millis(); // Record the start time
  while (millis() - START_TIME < DURATION) 
  {
    // Code inside loop runs while waiting (other tasks can be added here)
  }
}

void moveForward() {  
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, MAX_SPEED);

  analogWrite(MOTOR_B1, MAX_SPEED);  
  analogWrite(MOTOR_B2, 0);

}

//function to make robot go backwards
void moveBackwards() {
  brakeLight();
  analogWrite(MOTOR_A1, MAX_SPEED);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, MAX_SPEED);
}

//function to make robot go left
void moveLeft(){
  leftSignal();
  analogWrite(MOTOR_A1, MAX_SPEED);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, MAX_SPEED);  
  analogWrite(MOTOR_B2, 0);
}

//function to make robot go right
void moveRight(){
  rightSignal();
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, MAX_SPEED);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, MAX_SPEED);
}

//function to make robot stop
void stopMotor(){
  brakeLight();
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
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

    delay(500); // Wait for the specified time

    // Turn the pixels off
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();  // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time
}

void leftSignal() {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(70, 255, 0)); //Set left rear color to orange (G,R,B)
    pixels.setPixelColor(3, pixels.Color(70, 255, 0)); //Set left front color to orange (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time

    // Turn the pixels off
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time
}

