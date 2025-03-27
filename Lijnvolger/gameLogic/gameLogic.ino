#include <Adafruit_NeoPixel.h>

// Remove comment lines below to enable debugging
// #define DEBUG

#define NEO_PIXEL_PIN 13 //Attached to digital pin 13
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Motor pins
#define MOTOR_A1 6  // Left motor pin 1
#define MOTOR_A2 11   // Left motor pin 2
#define MOTOR_B1 10   // Right motor pin 1
#define MOTOR_B2 5   // Right motor pin 2

// Ultrasonic sensor pins
#define SERVO_PIN 12
#define ECHO_PIN 9
#define TRIG_PIN 8

// Start button pin
#define START_BUTTON_PIN 4

//range how the gripper open and closes. 360 degrees
#define OPENGRIP_VALUE 120
#define CLOSEGRIP_VALUE 50

unsigned long currentMillis = 0;
unsigned long previousActionMillis = 0;
unsigned long previousServoMillis = 0;
const int SERVO_INTERVAL = 20; // 20ms delay for the servo

boolean isGripClosed = false;

const int MAX_SPEED = 255;

const int OBSTACLE_THRESHOLD = 10; // Trigger distance of obstacle in cm
const int END_OF_LINE_TIMEOUT = 300; // Time in ms to determine it's an end square or crossing a line

const int NUM_SENSORS = 8;
const int SENSOR_PINS[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int thresholdLow = 0;
int thresholdHigh = 0;
bool calibrationComplete = false;
unsigned long allBlackStartTime = 0;
bool allBlackDetected = false;
bool onBlackLine = false;
int linesPast = 0;

// Variables for the states
enum robotState {
  PARKED, // Starting state, press button to go to START
  START, // Drive forward over 4 lines, then turn left to find the main line
  FOLLOWING_LINE, // Line following logic
  AVOIDING_OBSTACLE, // Avoid obstacle on the line
  END_OF_THE_LINE, // Detect black square at the end of the line
  FINDING_LINE, // Search for the line if it's lost
  END_SEQUENCE // Ending sequence, drop the cone and reverse out
};

robotState currentState = PARKED;  // Start in PARKED state
unsigned long stateStartTime = 0;
int avoidanceStep = 0;
int lastDirection = 0; // -1 (left), 1 (right), 0 (center)
int startStep = 0;
bool buttonPressed = false;
unsigned long startStep1DelayStartTime = 0; // Timer for delay in start step 1
unsigned long findLineMoveForwardStartTime = 0; // Timer for delay in findLine
unsigned long endReverseStartTime = 0; // Timer for delay in end reverse
int endSequenceStep = 0; // Step for the end sequence
unsigned long endSequenceStartTime = 0; // Timer for end sequence steps

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SERVO_PIN, OUTPUT);

  pixels.begin();
  Serial.begin(9600);

  // Initialize sensor min/max values
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    minValues[i] = 1023;  // Start with highest possible value
    maxValues[i] = 0;    // Start with lowest possible value
  }

  stop();

  // Set all LEDs to blue to indicate PARKED state
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
  }
  pixels.show();
}

void loop() {
  long distance = getDistance();

  buttonControl();
  calibrate();
  gripperControl();

  currentMillis = millis(); // Update currentMillis at the beginning of loop

  switch (currentState) {
    case PARKED:
      // Wait for button press
      stop();
      // Keep blue LEDs on for PARKED state
      for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 255));
      }
      pixels.show();
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
        alarm(); // Red warning lights
      }
      break;


    case AVOIDING_OBSTACLE:
      avoidObstacle(distance);
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
  }
}

void buttonControl() {
  // Check for button press
  if (digitalRead(START_BUTTON_PIN) == LOW && !buttonPressed) {
    buttonPressed = true;
    if (currentState == PARKED) {
      currentState = START;
      startStep = 0;
      stateStartTime = currentMillis;
      linesPast = 0; // Reset lines past when starting
    }
  }

  // Reset button state when released
  if (digitalRead(START_BUTTON_PIN) == HIGH) {
    buttonPressed = false;
  }
}

void calibrate() {
  int average = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = analogRead(SENSOR_PINS[i]);
    minValues[i] = min(reading, minValues[i]);
    maxValues[i] = max(reading, maxValues[i]);

    average += (minValues[i] + maxValues[i]) / 2;
  }

  average /= NUM_SENSORS;

  thresholdLow = average - 30;
  thresholdHigh = average + 30;
}

void start() {
  if (startStep == 0) {
    // Drive over horizontal lines for calibration
    // Yellow LEDs to indicate calibration step
    for (int i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 0));
    }
    pixels.show();

    drive(210, 218);
    // Count lines past
    int sensorReadings[NUM_SENSORS];
    int blackSensorCount = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorReadings[i] = analogRead(SENSOR_PINS[i]);
      // Count how many sensors detect black
      if (sensorReadings[i] >= thresholdHigh) {
        blackSensorCount++;
      }
    }

    // Detect transitions from not on a line to on a line
    if (blackSensorCount >= 6 && !onBlackLine) {
      onBlackLine = true;
      linesPast++;
    } else if (blackSensorCount < 4) { // Reset when clearly off the line
      onBlackLine = false;
    }

    // After passing 3 calibration lines
    if (linesPast >= 4) {
      onBlackLine = false;
      delay(300);
      isGripClosed = true;
      startStep = 1;
      stateStartTime = millis();
      linesPast = 0;
    }
  } else if (startStep == 1) {
    // Slow down and look for the main line
    // White LEDs to indicate looking for line
    for (int i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    }
    pixels.show();

    leftTurn(); // Turn left to find main line

    // Check if line is detected
    if (isLineDetected()) {
      // Go to FOLLOWING_LINE state
      currentState = FOLLOWING_LINE;
      stop(); // Stop turning when line is found
    }

    // If it's searching too long, try turning to find the line
    if (currentMillis - stateStartTime > 4000) {
      startStep = 2;
      stateStartTime = currentMillis;
      stop(); // Stop before turning in place
    }
  } else if (startStep == 2) {
    // Turn to find the line if not found yet
    leftTurn(); // Turn in place

    // Check if line is detected
    if (isLineDetected()) {
      Serial.println("Line found, Switching to line following mode");
      currentState = FOLLOWING_LINE;
      stop(); // Stop turning when line is found
    }

    // If it's searching too long, reset to step 1
    if (currentMillis - stateStartTime > 3000) {
      startStep = 1;
      stateStartTime = currentMillis;
      stop(); // Stop before restarting search
    }
  }
}

void avoidObstacle(long distance) {
  // State for obstacle avoidance
  switch (avoidanceStep) {
    case 0: // Turn right around the obstacle
      drive(200, 50);
      if (currentMillis - stateStartTime > 500) {
        avoidanceStep = 1;
        stateStartTime = currentMillis;
      }
      break;

    case 1: // Move forward while turning right
      drive(200, 150);
      rightSignal();

      // If we've moved for a sufficient time or distance is now safe
      if (currentMillis - stateStartTime > 1000 || (distance > OBSTACLE_THRESHOLD + 10 && distance > 0)) {
        avoidanceStep = 2;
        stateStartTime = currentMillis;
      }
      break;

    case 2: // Continue moving forward
      drive(200, 200);
      if (currentMillis - stateStartTime > 500) {
        avoidanceStep = 3;
        stateStartTime = currentMillis;
      }
      break;

    case 3: // Start turning left back towards the line
      drive(150, 200);
      leftSignal();

      // If it's turned for enough time or found the line
      if (currentMillis - stateStartTime > 1000 || isLineDetected()) {
        if (isLineDetected()) {
          // Line found, go back to line following
          currentState = FOLLOWING_LINE;
        } else {
          // Still need to search for the line
          currentState = FINDING_LINE;
          stateStartTime = currentMillis;
          findLineMoveForwardStartTime = currentMillis; // Initialize timer for findLine
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

  // Check if middle sensors detect the line
  if ((sensorReadings[3] >= thresholdHigh && sensorReadings[4] >= thresholdHigh) &&
      sensorReadings[0] <= thresholdLow && sensorReadings[7] <= thresholdLow) {
    lineDetected = true;
  }

  return lineDetected;
}

void findLine() {
  // Turn slowly in place to find the line
  leftTurn();

  // Check if the line is detected
  if (isLineDetected()) {
    currentState = FOLLOWING_LINE;
    stop(); // Stop turning when line is found
  }

  // If it's searching too long, move forward a bit and try again
  if (currentMillis - stateStartTime > 3000) {
    drive(220, 220);
    if (currentMillis - findLineMoveForwardStartTime >= 300) {
      stop();
      stateStartTime = currentMillis; // Reset the timer
      findLineMoveForwardStartTime = currentMillis; // Reset the move forward timer
    }
  } else {
    findLineMoveForwardStartTime = currentMillis; // Keep updating the move forward timer if not in the long search
  }
}

void follow() {
  int sensorReadings[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorReadings[i] = analogRead(SENSOR_PINS[i]);
  }

  // If all sensors are black for a longer period of time
  if (isEndOfLine()) {
    currentState = END_OF_THE_LINE;
    stateStartTime = currentMillis; // Set start time for the state
    return;
  }

  // If all sensors are black for a short time
  if (allBlackDetected) {
    drive(240, 240); // Move forward to cross horizontal line
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
    alarm();
  }
}

bool isEndOfLine() {
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

  // Check if most sensors detect black (at least 6 out of 8)
  if (blackSensorCount >= 6) {
    // If most sensors detect black, start a timer
    if (!allBlackDetected) {
      allBlackDetected = true;
      allBlackStartTime = currentMillis;
      return false; // Don't trigger end of line yet
    }
    // If most sensors are black for longer than the timeout
    else if (currentMillis - allBlackStartTime > END_OF_LINE_TIMEOUT) {
      return true;
    }

    return false;
  } else {
    // Not enough black sensors, reset
    allBlackDetected = false;
    return false;
  }
}

void endSequence() {
  switch (endSequenceStep) {
    case 0:
      // Drop the cone
      isGripClosed = false;
      // Reverse out after dropping
      reverse(255, 255);
      reverseLight();
      endSequenceStep++;
      endReverseStartTime = currentMillis;
      break;
    case 1:
      // Wait for reverse duration
      if (currentMillis - endReverseStartTime > 800) {
        stop();
        endSequenceStep++;
        endSequenceStartTime = currentMillis;
      }
      break;
    case 2:
      if (currentMillis - endSequenceStartTime > 1000) {
        // Set all LEDs to a steady green to indicate completion
        currentState = PARKED; // Return to parked state after completion
      }
      break;
  }
}

void drive(int SPEED_LEFT, int SPEED_RIGHT) {
  SPEED_LEFT = constrain(SPEED_LEFT, 0, MAX_SPEED);
  SPEED_RIGHT = constrain(SPEED_RIGHT, 0, MAX_SPEED);

  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, SPEED_LEFT);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, SPEED_RIGHT);
}

void reverse(int SPEED_LEFT, int SPEED_RIGHT) {
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

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

long getDistance() {
  // establish variables for duration of the ping, and the distance result
  // in centimeters:
  long duration, cm;

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
  duration = pulseIn(ECHO_PIN, HIGH, 20000);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  return cm;
}

void generatePulse(int angle) {
  int pulseWidth = map(angle, 0, 180, 544, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}

void gripperControl() {
  currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousActionMillis;

  // Keep servo active
  if (currentMillis - previousServoMillis >= SERVO_INTERVAL) {
    previousServoMillis = currentMillis;
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
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // GRB format - White reverse light
  pixels.setPixelColor(1, pixels.Color(255, 255, 255));
  pixels.show();
}

void greenLight() {
  pixels.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));  // GRB format - Green
  }
  pixels.show();
}

void alarm() {
  pixels.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 255));  // GRB format - Red alarm
  }
  pixels.show();
}