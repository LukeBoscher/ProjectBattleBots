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

const long OPEN_TIME = 0;            // Open grip at start
const long CLOSE_TIME = 1000;        // Close grip after 1s
const long OPEN_AGAIN_TIME = 2000;    // Open grip again after 2s
const long DRIVE_START_TIME = 3000;   // Start driving after 3s
const long GRAB_TIME = 5000;         // Grab cone after 4s 
const long HOLD_TIME = 7000;         // Stop and hold after 6s
const long DROP_TIME = 9000;        // Drop after 8s
const long DRIVE_AGAIN_TIME = 6000;  // Drive again after 5s

const int MAX_SPEED = 240;
const int TURN_SPEED = 150;

boolean isGripClosed = false;

const int NUM_SENSORS = 8;
const int SENSOR_PINS[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];

// Store last known direction: -1 (left), 1 (right), 0 (center)
int LAST_DIRECTION = 0;
bool LINE_FOUND = false;

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

  pixels.begin();
  Serial.begin(9600);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    minValues[i] = 1023;
    maxValues[i] = 0;
  }
  calibrateSensors();
  Serial.println("Beweeg de robot over de lijn voor kalibratie...");

  crossThreeLines();

  findLineOnLeft();
}

void loop() 
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


void crossThreeLines() {
  int linesCrossed = 0;
  bool onBlack = false;

  Serial.println("Crossing three lines...");
  while (linesCrossed < 2) {
    int blackSensors = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (analogRead(SENSOR_PINS[i]) < 500) { // Black detected
        blackSensors++;
      }
    }

    moveForward();

    if (blackSensors > 3) 
    { // More than 3 sensors see black
      if (!onBlack) 
      {
        Serial.print("Line crossed: ");
        Serial.println(linesCrossed);
        onBlack = true;
        delay(100); // Prevents false detections
      }
    } 
    else 
    {
      onBlack = false;
    }

    drive(150, 150); // Move forward slowly
  }

  Serial.println("Three lines crossed.");
  stop();
  delay(100);
}


void findLineOnLeft() {
  Serial.println("Searching for line on the left...");
  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // Yellow light when searching
  pixels.show();

  bool foundLine = false;

  for (int i = 0; i < 10; i++) 
  {

    for (int j = 0; j < NUM_SENSORS; j++) {
      if (analogRead(SENSOR_PINS[j]) < 500) { // Found a line
        foundLine = true;
        Serial.println("Line found on left!");
        LINE_FOUND = true;
        turnLeft();
        return;
      } 
    }
  }

  if (!foundLine) {
    Serial.println("No line found, adjusting...");
    drive(-100, -100); // Move back slightly
    delay(500);
    findLineOnLeft();
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

void hardCodedStart()
{
  moveForward();
  delay(2000);

  turnLeft();
  delay(500);

  moveForward();
  delay(1000);
}

void dodge (){
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
      stop();
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
      stop();
    }
    
    // Drop the cone after the specified time
    else if (elapsedTime >= DROP_TIME) 
    {
      isGripClosed = false;
    }
}

