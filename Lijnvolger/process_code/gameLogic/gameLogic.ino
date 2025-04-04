#include <Adafruit_NeoPixel.h>

// NeoPixel 
#define NEO_PIXEL_PIN 13 
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Motor pins
#define MOTOR_A1 6  // Left motor pin 1
#define MOTOR_A2 11   // Left motor pin 2
#define MOTOR_B1 10   // Right motor pin 1
#define MOTOR_B2 5   // Right motor pin 2

// Servo pin
#define SERVO_PIN 12

// Ultrasonic sensor pins
#define ECHO_PIN 9
#define TRIG_PIN 8

// Start button pin
#define START_BUTTON_PIN 4

//range how the gripper open and closes. 360 degrees
#define OPENGRIP_VALUE 100
#define CLOSEGRIP_VALUE 46

const int SERVO_INTERVAL = 10; // 10ms delay for the servo
boolean isGripClosed = false; // Start with open gripper

// Initialisation for millis
unsigned long currentMillis = 0; 
unsigned long servoMillis = 0;
unsigned long previousServoMillis = 0;
unsigned long allBlackStartTime = 0;
unsigned long stateStartTime = 0;
unsigned long endSequenceStartTime = 0; 
unsigned long endReverseStartTime = 0; 

const int MAX_SPEED = 255; // Maximum speed of the motor
const int BLACK_SQUARE_TIMEOUT = 300; // Time in ms to determine it's an end square or crossing a line
const int OBSTACLE_THRESHOLD = 15; // Trigger distance of obstacle in cm

const int NUM_SENSORS = 8; // Number of sensors
const int SENSOR_PINS[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

int minValues[NUM_SENSORS]; // Used to store minimum values of the line sensors
int maxValues[NUM_SENSORS]; // Used to store maximum values of the line sensors
int thresholdLow = 0; // Calculated threshold for the line sensors (average of all sensors - 30)
int thresholdHigh = 0; // Calculated threshold for the line sensors (average of all sensors + 30)
bool allBlackDetected = false; // Used to check if atleast 6 sensors detect black
int positiveReadings = 0; // Amount of positive readings from ultrasonic sensor, used in flagStart
int avoidanceStep = 0; // Step for the AVOIDING_OBSTACLE state
int lastDirection = 0; // Used when line is lost, -1 (left), 1 (right), 0 (center)
int startStep = 0; // Step for the START state
bool buttonPressed = false; // Start with button not pressed
int endSequenceStep = 0; // Step for the end sequence
long distance = 0; // Stores distance from object

// Variables for the states
enum robotState {
  PARKED, // Starting state, press button to go to START state
  START, // Drive forward over calibration lines, then turn left to find the main line
  FOLLOWING_LINE, // Line following logic
  AVOIDING_OBSTACLE, // Avoid obstacle on the line
  END_OF_THE_LINE, // Detect black square at the end of the line
  FINDING_LINE, // Search for the line if it's lost
  END_SEQUENCE, // Ending sequence, drop the cone and reverse out
  ENDED
};

robotState currentState = PARKED;  // Start in PARKED state

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SERVO_PIN, OUTPUT);

  pixels.begin(); // Initialize NeoPixels
  Serial.begin(9600); // Initialize serial printing

  // Initialize sensor min/max values
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    minValues[i] = 1023;  // Start with highest possible value
    maxValues[i] = 0;    // Start with lowest possible value
  }

  stop(); // Make sure the motors are stopped
}

void loop() {
  currentMillis = millis(); // Set currentMillis at the beginning of loop
  
  // Only measure distance if not in ENDED state to not interfere with other robots 
  if (currentState != ENDED) {
    distance = getDistance(); // Stores distance from object
  }
  
  flagStart(distance); // Function to start once flag is raised
  calibrate(); // Constantly calculate average light received by sensors
  gripperControl(); // Keep gripper active

  switch (currentState) {
    case PARKED:
      stop();
      parkingLight();
      break;

    case START:
      start();
      break;

    case FOLLOWING_LINE:
      follow();

      // Check for obstacles
      if (distance <= OBSTACLE_THRESHOLD && distance > 0) {
        // Obstacle detected, switch to obstacle avoidance mode
        currentState = AVOIDING_OBSTACLE;
        avoidanceStep = 0;
        stateStartTime = currentMillis;
      }
      break;

    case AVOIDING_OBSTACLE:
      avoidObstacle(distance);
      alarm(); // Red warning lights
      break;

    case END_OF_THE_LINE:
      endSequenceStep = 0; // Reset end sequence step
      endSequenceStartTime = currentMillis;
      currentState = END_SEQUENCE;
      break;

    case FINDING_LINE:
      findLine();
      break;

    case END_SEQUENCE:
      endSequence();
      break;

    case ENDED:
      stop();
      lightShow();
      break;
  }
}

// Function to start once flag is raised
void flagStart(long distance) {
  // Check for button press
  if (digitalRead(START_BUTTON_PIN) == LOW && !buttonPressed) {
    buttonPressed = true;
  }

  // Require 5 readings for better stability
  if(distance >= OBSTACLE_THRESHOLD && distance > 0 && buttonPressed == true) {
    positiveReadings++;
  }

  if(positiveReadings >= 5 && buttonPressed == true) {
    if(currentState == PARKED) {
      currentState = START;
      startStep = 0;
      stateStartTime = currentMillis;
      buttonPressed = false; // reset button
      positiveReadings = 0; // reset positiveReadings
    }
  }
}

void calibrate() {
  int average = 0; // Add up all average values from the sensors (minimum + maximum / 2)

  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = analogRead(SENSOR_PINS[i]);
    minValues[i] = min(reading, minValues[i]);
    maxValues[i] = max(reading, maxValues[i]);

    average += (minValues[i] + maxValues[i]) / 2;
  }

  average /= NUM_SENSORS; // Divide the average by the amount of sensors

  thresholdLow = average - 30; // Below thresholdLow = White
  thresholdHigh = average + 30; // Above thresholdHigh = Black
}

void start() {
  if (startStep == 0) {
    // Yellow LEDs to indicate calibration step
    for (int i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 150, 0));
    }
    pixels.show();

    // Drive over horizontal lines for calibration
    drive(205, 222);
  
    // After detecting a black square
    if (isBlackSquare()) {
      isGripClosed = true;
      startStep = 1;
      stateStartTime = millis();
    }

  } else if (startStep == 1) {
    // White LEDs to indicate looking for line
    for (int i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 150, 150));
    }
    pixels.show();

    leftTurn(); // Turn left to find main line

    // Check if line is detected
    if (isLineDetected()) {
      // Go to FOLLOWING_LINE state
      currentState = FOLLOWING_LINE;
      stop(); // Stop turning when line is found
    }
  } 
}

// Function to avoid obstacle on the line
void avoidObstacle(long distance) {
  switch (avoidanceStep) {
    case 0: // Turn right around the obstacle
      drive(220, 0);
      rightSignal();
      // 2 options, turn for 500ms or turn until obstacle is no longer in front
      if (currentMillis - stateStartTime > 500 || (distance > OBSTACLE_THRESHOLD + 10 && distance > 0)) {
        avoidanceStep = 1;
        stateStartTime = currentMillis;
      }
      break;

    case 1: // Move forward for 500 ms
      drive(220, 220);
      if (currentMillis - stateStartTime > 800) {
        avoidanceStep = 2;
        stateStartTime = currentMillis;
      }
      break;

    case 2: // Sharp left turn for 400 ms
      drive(0, 240);
      leftSignal();
      if (currentMillis - stateStartTime > 400) {
        avoidanceStep = 3;
        stateStartTime = currentMillis;
      }
      break;

    case 3: // Slight left turn to find the line
      drive(160, 255);
      leftSignal();


      if (currentMillis - stateStartTime > 4000 || isLineDetected()) {
        if (isLineDetected()) {
          // Line found, go back to line following
          currentState = FOLLOWING_LINE;
        } else {
          // If the line is not found within 4 seconds
          currentState = FINDING_LINE;
          stateStartTime = currentMillis;
        }
      }
      break;
  }
}

bool isLineDetected() {
  int sensorReadings[NUM_SENSORS];
  bool lineDetected = false;

  // Read sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorReadings[i] = analogRead(SENSOR_PINS[i]);
  }

  // Check if middle sensors detect the line and outer sensors detect white
  if ((sensorReadings[3] >= thresholdHigh && sensorReadings[4] >= thresholdHigh) &&
      sensorReadings[0] <= thresholdLow && sensorReadings[7] <= thresholdLow) {
    lineDetected = true;
  }

  return lineDetected;
}

void findLine() {
  // Turn in place to find the line
  leftTurn();

  // Check if the line is detected
  if (isLineDetected()) {
    currentState = FOLLOWING_LINE;
    stop(); // Stop turning when line is found
  }
}

void follow() {
  int sensorReadings[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorReadings[i] = analogRead(SENSOR_PINS[i]);
  }

  // If all sensors are black for a longer period of time
  if (isBlackSquare()) {
    currentState = END_OF_THE_LINE; // Switch state to END_OF_THE_LINE
    stateStartTime = currentMillis; // Set start time for the state
    return;
  }

  // If all sensors are black for a short time
  if (allBlackDetected) {
    drive(220, 225); // Move forward to cross horizontal line
    return;
  }

  // Line Following Logic
  if (sensorReadings[3] >= thresholdHigh && sensorReadings[4] >= thresholdHigh) {
    drive(255, 255);  // Move forward
    lastDirection = 0;  // Reset direction when moving straight
    greenLight();
  } else if (sensorReadings[4] >= thresholdHigh && sensorReadings[5] >= thresholdHigh) {
    drive(255, 165);  // Slight right
    lastDirection = -1;  // Remember last seen black was on the left
    rightSignal();
  } else if (sensorReadings[5] >= thresholdHigh && sensorReadings[6] >= thresholdHigh) {
    drive(255, 55);   // More right
    lastDirection = -1;
    rightSignal();
  } else if (sensorReadings[6] >= thresholdHigh) {
    drive(255, 5);    // sharp right
    lastDirection = -1;
    rightSignal();
  } else if (sensorReadings[2] >= thresholdHigh && sensorReadings[3] >= thresholdHigh) {
    drive(165, 255);  // Slight left
    lastDirection = 1;   // Remember last seen black was on the right
    leftSignal();
  } else if (sensorReadings[1] >= thresholdHigh && sensorReadings[2] >= thresholdHigh) {
    drive(55, 255);   // More left
    lastDirection = 1;
    leftSignal();
  } else if (sensorReadings[0] >= thresholdHigh && sensorReadings[1] >= thresholdHigh) {
    drive(5, 255);    // Sharp left
    lastDirection = 1;
    leftSignal();
  } else {
    // If line is lost, steer towards last known direction
    if (lastDirection == -1) {
      drive(255, 0);  // Turn right to search
    } else if (lastDirection == 1) {
      drive(0, 255);  // Turn left to search
    } else {
      drive(200, 200);  // Go forward slowly if no memory
    }
  }
}

// Function to detect a black square
bool isBlackSquare() {
  int sensorReadings[NUM_SENSORS];
  int blackSensorCount = 0;

  // Read all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorReadings[i] = analogRead(SENSOR_PINS[i]);
    // Count how many sensors detect black
    if (sensorReadings[i] >= thresholdHigh) {
      blackSensorCount++;
    }
  }

  // If most sensors detect black (at least 6 out of 8)
  if (blackSensorCount >= 6) {
    // Start a timer
    if (!allBlackDetected) {
      allBlackDetected = true;
      allBlackStartTime = currentMillis;
      return false; // Don't trigger end of line yet
    }
    // If most sensors are black for longer than the timeout
    else if (currentMillis - allBlackStartTime > BLACK_SQUARE_TIMEOUT) {
      return true;
    }

    return false;
  } else {
    // Not enough black sensors, reset
    allBlackDetected = false;
    return false;
  }
}

// Function to drop the cone on the black square
void endSequence() {
  switch (endSequenceStep) {
    case 0:
      stop();
      brakeLight();
      endSequenceStep++;
      endReverseStartTime = currentMillis;
      break;
    
    // After 200 ms, reverse
    case 1:
      if(currentMillis - endReverseStartTime > 200) {
        reverse(230,230);
        reverseLight();
        endSequenceStep++;
        endReverseStartTime = currentMillis;
      }
      break;

    // After 400 ms, stop
    case 2:
      if (currentMillis - endReverseStartTime > 400) {
        stop();
        brakeLight();
        endSequenceStep++;
        endSequenceStartTime = currentMillis;
      }
      break;

    // After 100 ms, open the gripper
    case 3:
      if(currentMillis - endReverseStartTime > 100) {
        isGripClosed = false;
        brakeLight();
        endSequenceStep++;
        endSequenceStartTime = currentMillis;
      }
      break;

    // After 100 ms, reverse
    case 4:
      if (currentMillis - endSequenceStartTime > 100) {
        reverse(230, 230);
        reverseLight();
        endSequenceStep++;
        endSequenceStartTime = currentMillis;
      }
      break;

    // After 2000ms, end the program
    case 5:
      if (currentMillis - endSequenceStartTime > 2000) {
        currentState = ENDED; // Display a lightshow
      }
  }
}

void drive(int SPEED_LEFT, int SPEED_RIGHT) {
  // Keep speed within limits (between 0 and 255)
  SPEED_LEFT = constrain(SPEED_LEFT, 0, MAX_SPEED);
  SPEED_RIGHT = constrain(SPEED_RIGHT, 0, MAX_SPEED);

  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, SPEED_LEFT);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, SPEED_RIGHT);
}

void reverse(int SPEED_LEFT, int SPEED_RIGHT) {
  // Keep speed within limits (between 0 and 255)
  SPEED_LEFT = constrain(SPEED_LEFT, 0, MAX_SPEED);
  SPEED_RIGHT = constrain(SPEED_RIGHT, 0, MAX_SPEED);

  analogWrite(MOTOR_A1, SPEED_LEFT);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, SPEED_RIGHT);
  analogWrite(MOTOR_B2, 0);
}

void leftTurn() {
  analogWrite(MOTOR_A1, 210);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 230);
}

void stop() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

// Function to calculate microseconds to centimeters
long microsecondsToCentimeters(long microseconds) {
  /* 
  The speed of sound is 340 m/s or 29 microseconds per centimeter.
  The ping travels out and back, so to find the distance of the object we
  take half of the distance travelled.
  */
  return microseconds / 29 / 2;
}

// Function to get distance to object
long getDistance() {
  /* 
  establish variables for duration of the ping, and the distance result
  in centimeters:
  */
  long duration, cm;

  /* 
  The PING is triggered by a HIGH pulse of 2 or more microseconds.
  Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  */
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  /*
  The same pin is used to read the signal from the PING))): a HIGH pulse
  whose duration is the time (in microseconds) from the sending of the ping
  to the reception of its echo off of an object.
  */
  duration = pulseIn(ECHO_PIN, HIGH, 20000);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  return cm;
}

// Function to set gripper at a certain angle
void generatePulse(int angle) {
  int pulseWidth = map(angle, 0, 180, 544, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}

// Function to keep gripper active
void gripperControl() {
  servoMillis = millis();

  // Only send a pulse every 10ms 
  if (servoMillis - previousServoMillis >= SERVO_INTERVAL) {
    previousServoMillis = servoMillis;
    if (isGripClosed) {
      generatePulse(CLOSEGRIP_VALUE);
    } else {
      generatePulse(OPENGRIP_VALUE);
    }
  }
}

void brakeLight() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 150, 0)); // GRB format - Red brake light
  pixels.setPixelColor(1, pixels.Color(0, 150, 0));
  pixels.show();
}

void rightSignal() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(1, pixels.Color(70, 255, 0)); // GRB format - Orange right signal
  pixels.setPixelColor(2, pixels.Color(70, 255, 0));
  pixels.show();
}

void leftSignal() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(70, 255, 0)); // GRB format - Orange left signal
  pixels.setPixelColor(3, pixels.Color(70, 255, 0));
  pixels.show();
}

void reverseLight() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(150, 150, 150)); // GRB format - White reverse light
  pixels.setPixelColor(1, pixels.Color(150, 150, 150));
  pixels.show();
}

void parkingLight() {
  pixels.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 150)); // GRB format - Blue 
  }
  pixels.show();
}

void greenLight() {
  pixels.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(150, 0, 0));  // GRB format - Green
  }
  pixels.show();
}

void alarm() {
  pixels.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));  // GRB format - Red alarm
  }
  pixels.show();
}

// Function for a light show, only used at the end
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