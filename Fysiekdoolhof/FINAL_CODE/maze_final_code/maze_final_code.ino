// I/O List
// | Pin   | Name              | Description                          |
// |-------|-------------------|--------------------------------------|
// | 6     | MOTOR_L1_PIN      | Left motor forward                  |
// | 11    | MOTOR_L2_PIN      | Left motor backward                 |
// | 10    | MOTOR_R1_PIN      | Right motor forward                 |
// | 5     | MOTOR_R2_PIN      | Right motor backward                |
// | 8     | TRIG_FRONT_PIN    | Ultrasonic sensor front trigger     |
// | 9     | ECHO_FRONT_PIN    | Ultrasonic sensor front echo        |
// | 3     | TRIG_LEFT_PIN     | Ultrasonic sensor left trigger      |
// | 2     | ECHO_LEFT_PIN     | Ultrasonic sensor left echo         |
// | 7     | TRIG_RIGHT_PIN    | Ultrasonic sensor right trigger     |
// | 4     | ECHO_RIGHT_PIN    | Ultrasonic sensor right echo        |
// | 12    | SERVO_PIN         | Servo control                       |
// | 13    | NEO_LED_PIN       | NeoPixel LED control                |
// | A0-A7 | SENSOR_PINS       | Light sensors                       |

// Constants
// | Name                  | Description                              |
// |-----------------------|------------------------------------------|
// | NUM_PIXELS            | Number of NeoPixel LEDs                 |
// | BASE_SPEED            | Base motor speed                        |
// | MAX_SPEED             | Maximum motor speed                     |
// | MIN_SPEED             | Minimum motor speed                     |
// | MIN_RIGHT_DISTANCE    | Minimum safe distance on the right      |
// | MIN_LEFT_DISTANCE     | Minimum safe distance on the left       |
// | MIN_FRONT_DISTANCE    | Minimum safe distance in the front      |
// | DEAD_END_THRESHOLD    | Threshold for detecting dead ends       |

#include <Adafruit_NeoPixel.h>

// Debugging
#define DEBUG

// NeoPixel Configuration
#define NEO_LED_PIN 13
#define NUM_PIXELS 4
#define DELAY_TIME_MS 500

// Servo Configuration
#define SERVO_PIN 12
#define GRIPPER_OPEN_PULSE 1550
#define GRIPPER_CLOSE_PULSE 1000

// Motor Pins
#define MOTOR_L1_PIN 6
#define MOTOR_L2_PIN 11
#define MOTOR_R1_PIN 10
#define MOTOR_R2_PIN 5

// Ultrasonic Sensor Pins
#define TRIG_FRONT_PIN 8
#define ECHO_FRONT_PIN 9
#define TRIG_LEFT_PIN 3
#define ECHO_LEFT_PIN 2
#define TRIG_RIGHT_PIN 7
#define ECHO_RIGHT_PIN 4

// Light Sensor Configuration
const int NUM_SENSORS = 8;
const int SENSOR_PINS[NUM_SENSORS] = { A7, A6, A5, A4, A3, A2, A1, A0 };

// Base Speed Settings
const int BASE_SPEED = 200;
const int MAX_SPEED = 255;
const int MIN_SPEED = 100;

// Safe Distance Thresholds
const float MIN_RIGHT_DISTANCE = 8.0;
const float MIN_LEFT_DISTANCE = 3.0;
const float MIN_FRONT_DISTANCE = 23.0;
const float DEAD_END_THRESHOLD = 5.0;

// Global Variables
Adafruit_NeoPixel strip(NUM_PIXELS, NEO_LED_PIN, NEO_RGB + NEO_KHZ800);
int lineFollowerCount = 0;
int deadzoneHigh = 0;
int deadzoneLow = 0;
float prevDistanceFront = 0;
float prevDistanceLeft = 0;
float prevDistanceRight = 0;
int lightSensorValues[NUM_SENSORS];
int minSensorValues[NUM_SENSORS];
int maxSensorValues[NUM_SENSORS];
int sensorReadings[NUM_SENSORS];
bool startMaze = true;
bool turning = true;
float initialDistanceLeft = 0;
bool mazeActive = false;
bool start = false;
bool reverse = false;

void setup() {
  // Initialize Motor Pins
  pinMode(MOTOR_L1_PIN, OUTPUT);
  pinMode(MOTOR_L2_PIN, OUTPUT);
  pinMode(MOTOR_R1_PIN, OUTPUT);
  pinMode(MOTOR_R2_PIN, OUTPUT);

  // Initialize Ultrasonic Sensor Pins
  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);
  pinMode(TRIG_LEFT_PIN, OUTPUT);
  pinMode(ECHO_LEFT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT);
  pinMode(ECHO_RIGHT_PIN, INPUT);

  // Initialize Light Sensor Pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // Initialize Servo
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  // Initialize Serial Communication
  Serial.begin(9600);

  // Initialize NeoPixel
  strip.begin();
  strip.show();

  stopMotors();
}

void driveMotors(int speedLeft, int speedRight) {
  if (speedLeft >= 0) {
    analogWrite(MOTOR_L1_PIN, 0);
    analogWrite(MOTOR_L2_PIN, speedLeft);
  } else {
    analogWrite(MOTOR_L1_PIN, -speedLeft);
    analogWrite(MOTOR_L2_PIN, 0);
  }

  if (speedRight >= 0) {
    analogWrite(MOTOR_R1_PIN, 0);
    analogWrite(MOTOR_R2_PIN, speedRight);
  } else {
    analogWrite(MOTOR_R1_PIN, -speedRight);
    analogWrite(MOTOR_R2_PIN, 0);
  }

  if (abs(speedLeft - speedRight) > 0) {
    if (speedRight > speedLeft) {
      controlLights("left");
    } else {
      controlLights("right");
    }
  } else if (abs(speedLeft - speedRight) < 10) {
    if (speedRight >= 0 || speedLeft >= 0) {
      controlLights("regular");
    } else if (speedRight < 0 || speedLeft < 0) {
      controlLights("brake");
    }
  }
}

void stopMotors() {
  analogWrite(MOTOR_L1_PIN, 0);
  analogWrite(MOTOR_L2_PIN, 0);
  analogWrite(MOTOR_R1_PIN, 0);
  analogWrite(MOTOR_R2_PIN, 0);
}

float measureDistance(const String& direction) {
  int trigPin, echoPin;

  if (direction == "front") {
    trigPin = TRIG_FRONT_PIN;
    echoPin = ECHO_FRONT_PIN;
  } else if (direction == "left") {
    trigPin = TRIG_LEFT_PIN;
    echoPin = ECHO_LEFT_PIN;
  } else if (direction == "right") {
    trigPin = TRIG_RIGHT_PIN;
    echoPin = ECHO_RIGHT_PIN;
  } else {
    return -1;  // Invalid input
  }

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  return (distance > 50) ? 50 : distance;
}

void moveServo(int pulseWidth) {
  unsigned long startTime = millis();
  while (millis() - startTime < 500) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(20000 - pulseWidth);
  }
}

void readLightSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    lightSensorValues[i] = analogRead(SENSOR_PINS[i]);
  }
}

void checkAndReverse(float& prevDistanceFront, float& prevDistanceLeft, float& prevDistanceRight) {
  reverse = false;
  float distanceFront = measureDistance("front");
  float distanceLeft = measureDistance("left");
  float distanceRight = measureDistance("right");

  static int countFront = 0;
  static int countLeft = 0;
  static int countRight = 0;

  if (abs(distanceFront - prevDistanceFront) <= 1 && distanceFront < 50) {
    countFront++;
  } else {
    countFront = 0;
  }

  if (abs(distanceLeft - prevDistanceLeft) <= 1 && distanceLeft < 50) {
    countLeft++;
  } else {
    countLeft = 0;
  }

  if (abs(distanceRight - prevDistanceRight) <= 1 && distanceRight < 50) {
    countRight++;
  } else {
    countRight = 0;
  }

  if (countFront >= 15 || countLeft >= 15 || countRight >= 15) {
    if (countLeft >= 10) {
      driveMotors(-200, -255);
    }
    if (countRight >= 10) {
      driveMotors(-255, -200);
    }
    if (countFront >= 10) {
      driveMotors(-255, -255);
    }

    delay(500);

    countFront = 0;
    countLeft = 0;
    countRight = 0;
    reverse = true;
  }

  prevDistanceFront = distanceFront;
  prevDistanceLeft = distanceLeft;
  prevDistanceRight = distanceRight;
}

void turn180(float distanceFront, float distanceLeft, float distanceRight) {
  unsigned long currentMillis = millis();
  float initialLeft = distanceLeft;
  float initialRight = distanceRight;

  while (currentMillis < 800) {
    if (currentMillis < 500) {
      stopMotors();
    }
    if (currentMillis < 800) {
      driveMotors(-255, -255);
    }
  }

  while (distanceFront < 30 || abs(distanceRight - distanceLeft) < 5) {
    distanceFront = measureDistance("front");
    distanceLeft = measureDistance("left");
    distanceRight = measureDistance("right");
    if (initialRight > initialLeft) {
      driveMotors(150, -220);
    } else {
      driveMotors(-220, 150);
    }
  }
}

void mazeLogic() {
  float distanceFront = measureDistance("front");
  float distanceLeft = measureDistance("left");
  float distanceRight = measureDistance("right");

  checkAndReverse(prevDistanceFront, prevDistanceLeft, prevDistanceRight);

  if (reverse) {
    float distanceFront = measureDistance("front");
    if (distanceFront < MIN_FRONT_DISTANCE) {
      distanceFront = 15;
    }
    float distanceLeft = measureDistance("left");
    float distanceRight = measureDistance("right");
  }

  int speedLeft = BASE_SPEED;
  int speedRight = BASE_SPEED;

  if (distanceFront <= 15 && distanceLeft <= 15 && distanceRight <= 15) {
    turn180(distanceFront, distanceLeft, distanceRight);
    return;
  }

  if (distanceFront > MIN_FRONT_DISTANCE) {
    if (distanceRight > 20) {
      speedLeft = MAX_SPEED;
      speedRight = 100;
    } else if (distanceRight < 20 && distanceFront < 15) {
      speedLeft = 100;
      speedRight = MAX_SPEED;
    } else if (distanceRight < 3) {
      speedLeft = 100;
      speedRight = MAX_SPEED;
    } else if (distanceLeft < 3) {
      speedLeft = MAX_SPEED;
      speedRight = 100;
    } else if (distanceRight > 9) {
      speedLeft = MAX_SPEED;
      speedRight = 190;
    } else if (distanceRight < 9) {
      speedLeft = 190;
      speedRight = MAX_SPEED;
    } else {
      speedLeft = 253;
      speedRight = 255;
    }
  } else if ((distanceRight - distanceLeft) > 3 && distanceRight > 20) {
    speedLeft = 255;
    speedRight = 25;
  } else if ((distanceLeft - distanceRight) > 3 && distanceLeft > 20) {
    speedLeft = 10;
    speedRight = 255;
  }

  driveMotors(speedLeft, speedRight);
}

void calibrateLineSensor() {
  static bool firstRun = true;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorReadings[i] = analogRead(SENSOR_PINS[i]);

    if (firstRun) {
      minSensorValues[i] = sensorReadings[i];
      maxSensorValues[i] = sensorReadings[i];
    }

    minSensorValues[i] = min(sensorReadings[i], minSensorValues[i]);
    maxSensorValues[i] = max(sensorReadings[i], maxSensorValues[i]);
  }

  firstRun = false;

  int sum = 0;
  for (int j = 0; j < NUM_SENSORS; j++) {
    sum += (minSensorValues[j] + maxSensorValues[j]) / 2;
  }
  int avgThreshold = sum / NUM_SENSORS;

  deadzoneHigh = avgThreshold + 30;
  deadzoneLow = avgThreshold - 30;
}

void lineFollower() {
  calibrateLineSensor();

  if (lineFollowerCount == 1) {
    mazeLogic();
  } else if (sensorReadings[4] >= deadzoneHigh && sensorReadings[5] >= deadzoneHigh) {
    driveMotors(214, 255);
    lineFollowerCount = 1;
  } else if (sensorReadings[5] >= deadzoneHigh && sensorReadings[6] >= deadzoneHigh) {
    driveMotors(255, 170);
    lineFollowerCount = 1;
  } else if (sensorReadings[6] >= deadzoneHigh && sensorReadings[7] >= deadzoneHigh) {
    driveMotors(255, 120);
    lineFollowerCount = 1;
  } else if (sensorReadings[3] >= deadzoneHigh && sensorReadings[4] >= deadzoneHigh) {
    driveMotors(170, 255);
    lineFollowerCount = 1;
  } else if (sensorReadings[2] >= deadzoneHigh && sensorReadings[3] >= deadzoneHigh) {
    driveMotors(120, 255);
    lineFollowerCount = 1;
  } else if (sensorReadings[1] >= deadzoneHigh && sensorReadings[2] >= deadzoneHigh) {
    driveMotors(120, 255);
    lineFollowerCount = 1;
  } else {
    if (lineFollowerCount == 1) {
      return;
    }
  }

  delay(50);
}

void setLight(int boven, int onder, bool active, int color1[3], int color2[3]) {
    static unsigned long previousMillis = 0;
    static bool ledState = false;  // Retain state across calls
    unsigned long currentMillis = millis();

    if (active) {  // Only blink if active is true
        if (currentMillis - previousMillis >= 250) {  
            previousMillis = currentMillis;
            ledState = !ledState;  // Toggle state
        }
    } else {
        ledState = false;
        previousMillis = 0;  // Turn off LED when blinking stops
    }

    if (ledState) {
        strip.setPixelColor(boven, strip.Color(color1[0], color1[1], color1[2]));  // ON state
        strip.setPixelColor(onder, strip.Color(color1[0], color1[1], color1[2]));  // ON state
    } else {
        strip.setPixelColor(boven, strip.Color(color2[0], color2[1], color2[2]));  // OFF state
        strip.setPixelColor(onder, strip.Color(color2[0], color2[1], color2[2]));  // OFF state
    }

    strip.show();  // Update the strip to reflect changes
}

// General-purpose blinkers for front, reverse, left, or right
void controlLights(String direction) {
    strip.clear();

    // Define color settings
    int orange[3] = {255, 69, 0};
    int red[3] = {150, 0, 0};
    int white[3] = {100, 100, 100};

    if (direction == "left") {
        setLight(0, 3, true, orange, red);  // Left blinker
        strip.setPixelColor(1, strip.Color(red[0], red[1], red[2]));  // Red on second pixel
        strip.setPixelColor(2, strip.Color(white[0], white[1], white[2]));  // White on third pixel
    } 
    else if (direction == "right") {
        setLight(1, 2, true, orange, red);  // Right blinker
        strip.setPixelColor(0, strip.Color(red[0], red[1], red[2]));  // Red on first pixel
        strip.setPixelColor(3, strip.Color(white[0], white[1], white[2]));  // White on fourth pixel
    } 
    else if (direction == "brake") {
        setLight(0, 1, false, red, white);  // Brake light
        strip.setPixelColor(2, strip.Color(white[0], white[1], white[2]));  // White on third pixel
        strip.setPixelColor(3, strip.Color(white[0], white[1], white[2]));  // White on fourth pixel
    } 
    else if (direction == "regular") {
        setLight(0, 1, false, red, white);  // Regular light
        strip.setPixelColor(2, strip.Color(white[0], white[1], white[2]));  // White on third pixel
        strip.setPixelColor(3, strip.Color(white[0], white[1], white[2]));  // White on fourth pixel
    }

    strip.show();  // Update the strip to reflect changes
}

void loop() {
  while (!start) {
    float distanceFront = measureDistance("front");
    if (distanceFront < 30 && distanceFront) {
      start = true;
    }
  }

  calibrateLineSensor();

  if (initialDistanceLeft == 0) {
    initialDistanceLeft = measureDistance("left");
    delay(800);
    driveMotors(252, 255);
    moveServo(GRIPPER_OPEN_PULSE);
    startMaze = true;
  }

  float distanceLeft = measureDistance("left");

  if (startMaze) {
    if (distanceLeft > initialDistanceLeft + 10) {
      startMaze = false;
    } else {
      driveMotors(240, 255);
    }
  } else {
    if (!mazeActive) {
      delay(300);
      moveServo(GRIPPER_CLOSE_PULSE);
      if (turning) {
        driveMotors(-255, 255);
        delay(500);
        driveMotors(253, 255);
        delay(100);
        mazeActive = true;
        turning = false;
      }
    }
    lineFollower();
  }
}