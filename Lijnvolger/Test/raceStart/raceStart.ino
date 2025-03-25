#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NEO_PIXEL_PIN 8 //Attached to digital pin 8
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

int turnCounter = 0; // Count consecutive turns
const int MAX_TURNS = 5; // Adjust this number based on testing

const int MOTOR_A1 = 10;  // Left motor pin 1
const int MOTOR_A2 = 5;   // Left motor pin 2
const int MOTOR_B1 = 6;   // Right motor pin 1
const int MOTOR_B2 = 9;   // Right motor pin 2

const int MOTOR_SENSOR1 = A4;
const int MOTOR_SENSOR2 = A5;

//yellow wire is trigger
const int TRIG_PIN = 13;

//green wire is echo 
const int ECHO_PIN = 12; //ECHO_PIN

const int START_BUTTON_PIN = 2;

//trigger distance of obstacle in cm
const int OBSTACLETHRESHOLD = 10;  

const int TRANSMIT_PIN = 11; 

const int SERVO_PIN = 3; 

// Range how the gripper open and closes. 360 degrees

int OPENGRIP_VALUE = 120;
int CLOSEGRIP_VALUE = 50;
unsigned long currentMillis = 0;
unsigned long previousActionMillis = 0;
unsigned long previousServoMillis = 0;
const long SERVO_INTERVAL = 20; // 20 ms delay

// Timing Constants
const long CLOSE_TIME = 1000;           // Close grip after 1s
const long GRAB_TIME = 2000;            // Grab cone after 1s
const long TURN_TIME = 4000;            // Turn left starts at 4s
const long HOLD_TIME = 3000;            // Hold the cone in place for 3s
const long FOLLOW_LINE_TIME = 5000;     // Follow the line for a while before dropping
const long DROP_TIME = 10000;           // Drop cone after 10s (time since pickup start)

const int MAX_SPEED = 240;
const int SLOW_SPEED = 200;
const int TURN_SPEED = 150;

boolean isGripClosed = false;

const int NUM_SENSORS = 8;
const int SENSOR_PINS[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];

// Store last known direction: -1 (left), 1 (right), 0 (center)
int LAST_DIRECTION = 0;
bool LINE_FOUND = false;

bool buttonPressed = false;


void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(MOTOR_SENSOR1, INPUT);
  pinMode(MOTOR_SENSOR2, INPUT);
  pinMode(TRANSMIT_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);


  pixels.begin();
  Serial.begin(9600);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    minValues[i] = 1023;
    maxValues[i] = 0;
  }
  calibrateSensors();
  Serial.println("Beweeg de robot over de lijn voor kalibratie...");
  raceStart(); 

}

void loop() 
{
  //followLine();
  pickUp();
}

void followLine()
{
    if(!LINE_FOUND)
    {
      findLine();
      return;
    }

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
  
  int DEADZONE_LOW = AVG_VALUE - 30;
  int DEADZONE_HIGH = AVG_VALUE + 30;

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
  else if (!BLACK_LINE_DETECTED)
  {
    LAST_DIRECTION = 0;
    searchLine();
  }  

      // If stuck in a loop, force a change
    if (turnCounter > MAX_TURNS) {
        Serial.println("Loop detected! Reversing...");
        drive(-150, -150); // Move backward slightly
        delay(500);
        turnCounter = 0;
    }
}

void calibrateSensors()
 {
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < 200; i++) {  // Sample multiple times
    for (int j = 0; j < NUM_SENSORS; j++) {
      int val = analogRead(SENSOR_PINS[j]);
      minValues[j] = min(minValues[j], val);
      maxValues[j] = max(maxValues[j], val);
    }
    delay(5);
  }
  Serial.println("Calibration complete.");
}



void drive(int SPEED_LEFT, int SPEED_RIGHT) {
  SPEED_LEFT = constrain(SPEED_LEFT, 0, MAX_SPEED);
  SPEED_RIGHT = constrain(SPEED_RIGHT, 0, MAX_SPEED);

  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, SPEED_LEFT);
  analogWrite(MOTOR_B1, SPEED_RIGHT);
  analogWrite(MOTOR_B2, 0);
}

void findLine()
{
  Serial.println("Searching for line");
  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // Yellow light when searching
  pixels.show();

  for(int i = 0; i < 10; i++)
  {
    drive(150, 0);
    delay(200);
    drive(0, 150);
    delay(200);

    int sensorReading = analogRead(SENSOR_PINS[4]);
    if(sensorReading < 500)
    {
      Serial.print("Line Found");
      LINE_FOUND = true;
      return;
    }
  }
}

void searchLine()
{
  Serial.println("Line Lost Searching ...");
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red when lost
  pixels.show();

    for (int i = 0; i < 10; i++) {
    drive(255, 0);
    delay(300);
    drive(0, 255);
    delay(300);

    for (int j = 0; j < NUM_SENSORS; j++) {
      if (analogRead(SENSOR_PINS[j]) < 500) {
        return;
      }
    }
  }
    // Reverse and try again
  drive(-100, -100);
  delay(300);
  findLine(); 
}

void raceStart() 
{
  int linesCrossed = 0;
  bool onBlack = false;
  bool onSquare = false;  // To track when the robot is on the black square

  Serial.println("Crossing three lines...");

  while (linesCrossed < 3) { // Continue until 2 lines are crossed
    int blackSensors = 0;
    
    // Check how many sensors detect black
    for (int i = 0; i < NUM_SENSORS; i++) {
      int sensorValue = analogRead(SENSOR_PINS[i]);
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(sensorValue); // Debug print to check sensor readings

      if (sensorValue < 600) { // Adjusted threshold to detect black
        blackSensors++;
      }

    }

    // Debug print to see how many sensors are detecting black
    Serial.print("Black Sensors Detected: ");
    Serial.println(blackSensors);

    // Check if the robot is on a black line
    if (blackSensors > 0) { // Any sensor detects black
      if (!onBlack) { // Transition from white to black
        linesCrossed++; // Count the line
        Serial.print("Line crossed: ");
        Serial.println(linesCrossed);
        onBlack = true;
        delay(200); // Prevent false detections
      }
    } 
    else {
      onBlack = false; // Reset when back on white
      onSquare = false; // No longer on the black square
    }

    if (linesCrossed < 2) 
    {
      slowMoveforward();
      delay(100);

    }

    if(linesCrossed >= 3)
    {
      slowMoveforward();
      delay(200);
      if (analogRead(SENSOR_PINS[0]) < 500 || analogRead(SENSOR_PINS[1]) < 500 || analogRead(SENSOR_PINS[2]) < 500) 
      {
        stop();
        delay(5000);
        pickUp();



        Serial.println("Robot is on the black square.");
        Serial.println("Pick-up triggered after crossing the black square.");
      }
      else
      {
        slowMoveforward();
        delay(100);
      }
    }

    // After the 2nd line is crossed, check if robot leaves the black square

  }
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

void slowMoveforward() 
{  
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, SLOW_SPEED);
  analogWrite(MOTOR_B1, SLOW_SPEED);  
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



bool obstacleDetected (){
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long DURATION, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  DURATION = pulseIn(ECHO_PIN, HIGH, 20000);

  // convert the time into a distance
  inches = microsecondsToInches(DURATION);
  cm = microsecondsToCentimeters(DURATION);


  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(100);

  if (DURATION == 0) 
  {
  Serial.println("No echo received");
  return;
  }

  moveForward();


  if (cm <= OBSTACLETHRESHOLD){
    Serial.println("Obstacle detected");
    }
}

void dodge()
{
      stop();
    delay(300); // When robot sees obstacle robot motor stops

    moveBackwards(); //robot goes backwards
    delay(300);

    turnLeft(); // robot turns left
    delay(450);

    moveForward(); //robot goes backwards
    delay(500);

    turnRight(); // robot turns right
    delay(500);

    moveForward(); //robot goes backwards
    delay(1500);

    turnRight(); // robot turns left
    delay(500);

    moveForward(); // robot moves forward again
    delay(500);

    turnLeft(); // robot turns left
    delay(450);
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

void generatePulse(int angle) {
  int pulseWidth = map(angle, 0, 180, 544, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}


void pickUp() {
    // Reset previousActionMillis to 0 or current time when the pickup starts
    if (previousActionMillis == 0) {
        previousActionMillis = millis(); // Set start time
    }

    currentMillis = millis(); // Current time for the pick-up process

    unsigned long elapsedTime = currentMillis - previousActionMillis;

    Serial.print("Elapsed Time: ");
    Serial.println(elapsedTime);

    // Open the grip at the start
    if (elapsedTime < CLOSE_TIME && isGripClosed) {
        isGripClosed = false;
        generatePulse(OPENGRIP_VALUE); // Open immediately
        Serial.println("Open Gripper");
    }
    // Grab the cone
    else if (elapsedTime >= GRAB_TIME && !isGripClosed) {
        stop();
        isGripClosed = true;
        generatePulse(CLOSEGRIP_VALUE); // Close immediately
        Serial.println("Closing Gripper");
        previousActionMillis = millis(); // Reset timer after grabbing
    }
    // Turn left while holding the cone
    else if (elapsedTime >= GRAB_TIME && elapsedTime < GRAB_TIME + TURN_TIME) {
        turnLeft();
    }
    // Hold the cone in place for a few seconds after turning
    else if (elapsedTime >= GRAB_TIME + TURN_TIME && elapsedTime < GRAB_TIME + TURN_TIME + HOLD_TIME) {
        stop();
    }
    // Follow the line while holding the cone
    else if (elapsedTime >= GRAB_TIME + TURN_TIME + HOLD_TIME && elapsedTime < GRAB_TIME + TURN_TIME + HOLD_TIME + FOLLOW_LINE_TIME) {
        followLine();
    }
    // Drop the cone after a certain period while still following the line
    else if (elapsedTime >= GRAB_TIME + TURN_TIME + HOLD_TIME + FOLLOW_LINE_TIME && elapsedTime < GRAB_TIME + TURN_TIME + HOLD_TIME + FOLLOW_LINE_TIME + DROP_TIME) {
        drop();  // Drop the cone while following the line
    }
    // Continue following the line after dropping the cone
    else {
        followLine();
    }
}



void drop() {
    // Release the grip to drop the cone
    isGripClosed = false;
    previousActionMillis = millis();  // Reset timer after dropping the cone
    // Additional code for dropping the cone (e.g., servo or motor movement if needed)
}