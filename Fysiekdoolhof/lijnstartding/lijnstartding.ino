#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NeoLED 13
#define NUM_PIXELS 4
#define DELAY_TIME 500

// Servo
#define SERVO 12
#define GRIPPER_OPEN 1550
#define GRIPPER_CLOSE 1000

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

// Motor pins
#define motor_L1 6
#define motor_L2 11
#define motor_R1 10
#define motor_R2 5

// Ultrasonic sensor pins
#define ECHO 9
#define TRIG 8
#define TRIG_LINKS 3
#define ECHO_LINKS 2
#define TRIG_RECHTS 7
#define ECHO_RECHTS 4

// Motor sensor
int cws1 = 0, cws2 = 0;
unsigned long previousMillis = 0;
const long interval = 1000;
const int fullspeedLinksVooruit = 214;
const int fullspeedLinksAchteruit = -241.5;

//
int lineFollowerCount = 0;

int deadzonehigh = 0;
int deadzonelow = 0;

// Light sensors
const int numSensors = 8;
const int sensorPins[numSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};
int lichtsensorWaarden[numSensors];  // Define the array to store light sensor readings
int minValues[numSensors];
int maxValues[numSensors];
int sensorReadings[numSensors];


// Global variables for line-following logic
int lastDirection = 0;
int blackCount = 0;
int whiteCount = 0;
bool blackDetected = false;
bool whiteDetected = false;

void setup() {
  // Motor pins
  pinMode(motor_L1, OUTPUT);
  pinMode(motor_L2, OUTPUT);
  pinMode(motor_R1, OUTPUT);
  pinMode(motor_R2, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG_LINKS, OUTPUT);
  pinMode(ECHO_LINKS, INPUT);
  pinMode(TRIG_RECHTS, OUTPUT);
  pinMode(ECHO_RECHTS, INPUT);

  // Light sensor pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Servo
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);

  Serial.begin(9600);

  // NeoPixel setup
  strip.begin();
  strip.show();

  stop();
}

// Function to move motors
void drive(int speedLeft, int speedRight) {
  if (speedLeft >= 0) {
    analogWrite(motor_L1, 0);
    analogWrite(motor_L2, speedLeft);
  } else {
    analogWrite(motor_L1, -speedLeft);
    analogWrite(motor_L2, 0);
  }

  if (speedRight >= 0) {
    analogWrite(motor_R1, 0);
    analogWrite(motor_R2, speedRight);
  } else {
    analogWrite(motor_R1, -speedRight);
    analogWrite(motor_R2, 0);
  }
}

// Stop motors
void stop() {
  analogWrite(motor_L1, 0);
  analogWrite(motor_L2, 0);
  analogWrite(motor_R1, 0);
  analogWrite(motor_R2, 0);
}

// Measure distance using ultrasonic sensors
float measureDistance(String kant) {
  int trig, echo;

  if (kant == "voor") {
    trig = TRIG;
    echo = ECHO;
  } else if (kant == "links") {
    trig = TRIG_LINKS;
    echo = ECHO_LINKS;
  } else if (kant == "rechts") {
    trig = TRIG_RECHTS;
    echo = ECHO_RECHTS;
  } else {
    return -1;  // Invalid input
  }

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  float duration = pulseIn(echo, HIGH);
  float distance = (duration * 0.0343) / 2;
  if (distance > 50) {
    distance = 50;
  }

  Serial.print("Distance (");
  Serial.print(kant);
  Serial.print("): ");
  Serial.println(distance);

  return distance;
}

// Move servo
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

    timer = millis() + 20;  // 20ms interval for servo
  }
}

// Read light sensors
void readLightSensors() {
  for (int i = 0; i < numSensors; i++) {
    lichtsensorWaarden[i] = analogRead(sensorPins[i]);
  }
}

// Print light sensor values
void printLightSensorValues() {
  for (int i = 0; i < 8; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(lichtsensorWaarden[i]);
  }
}

// White detection logic
void checkWhiteDetected() {
  if (blackDetected) {
    bool allWhite = true;
    if (whiteCount < 1) 
    {
      whiteDetected = true;
      whiteCount++;
      return;
    }
    for (int i = 2; i < 5; i++) {
      if (sensorReadings[i] >= deadzonehigh) {  // Threshold value for white
        allWhite = false;
        break;
      }
    }

    if (allWhite) {
      whiteDetected = true;
      blackDetected = false;
      Serial.print("WhiteCount: ");
      Serial.println(whiteCount);
    }
  }
  return;
}

// Black detection logic
void checkBlackDetected() {
  Serial.println("Checking for black detection..."); // Debugging line
  if (whiteDetected) {
    bool allBlack = true;
    for (int i = 2; i < 5; i++) {
      if (sensorReadings[i] <= deadzonehigh) {  // Threshold value for black
        allBlack = false;
        Serial.println("not black");
        break;
      }
    }

    if (allBlack) {
      blackCount++;
      Serial.print("Black Count: ");
      Serial.println(blackCount);
      whiteDetected = false;
      blackDetected = true;
      // Debugging: Print black and white counts
      Serial.print("Black Count: ");
      Serial.println(blackCount);
    }
  }
}


float prevDistanceFront = 0;


void checkAndReverse(float &prevDistanceFront) {
    float distanceFront = measureDistance("voor");
    static int countFront = 0;  // Teller behouden tussen function calls

    // Controleer of de huidige afstand ongeveer gelijk is aan de vorige met een marge van ±2
    if (abs(distanceFront - prevDistanceFront) <= 2 && distanceFront < 50) {
        countFront++;

        if (countFront >= 30) {
            Serial.println("Obstacle detected or stuck: moving back slightly...");
            drive(-255, -255);  // Rijdt een klein stukje achteruit
            delay(300);         // Wacht even
            drive(0, 0);
            countFront = 0;  // Reset teller na achteruitrijden
        }
    } else {
        countFront = 0;  // Reset teller als de afstand wél verandert
    }

    prevDistanceFront = distanceFront;  // Update vorige afstand
}






// **PID Gains (Tweak as needed)**
float Kp = 6.0;  // Increase P gain for stronger reaction
float Ki = 0.05; // Reduce I gain to avoid drifting
float Kd = 2.0;  // Increase D gain for better reaction speed

float lastError = 0;
float integral = 0;

// **Base Speed Settings**
int baseSpeed = 200;
int maxSpeed = 255;
int minSpeed = 100;

// **Safe Distance Thresholds**
const float minRightDistance = 6.0;  // If closer than this, must turn left
const float minLeftDistance = 3.0;   // If closer than this, must turn right
const float minFrontDistance = 17.0; // If closer than this, stop or reverse
const float deadEndThreshold = 5.0;  // Threshold to detect if it's completely surrounded

// **PID Function for Steering Correction**
int PIDControl(float error) {
  integral += error;
  float derivative = error - lastError;
  lastError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void mazeLogic() {
  float distanceFront = measureDistance("voor");
  float distanceLeft = measureDistance("links");
  float distanceRight = measureDistance("rechts");

  // **Constants for navigation**
  const float baseSpeed = 200;
  const float minSpeed = 100;
  const float maxSpeed = 255;
  const float minFrontDistance = 20;
  const float minRightDistance = 10;
  const float minLeftDistance = 10;
  static float prevDistanceFront = 0;

  // **Calculate PID Error for Steering (Right Preference)**
  float error = (distanceRight - distanceLeft);
  int correction = PIDControl(error);

  // **Calculate Wheel Speeds**
  int speedLeft = baseSpeed - correction;
  int speedRight = baseSpeed + correction;

  // **Keep Speeds Within Limits**
  speedLeft = constrain(speedLeft, minSpeed, maxSpeed);
  speedRight = constrain(speedRight, minSpeed, maxSpeed);

  checkAndReverse(prevDistanceFront);

  if (distanceFront <= minFrontDistance && distanceLeft <= 15 && distanceRight <= 15) {
      drive(-255, 255);  // Reverse and turn left
      delay(300);
      if (distanceLeft > 12) {
          drive(241, -254);
      } else {
          drive(-241, 254);
      }
      delay(1000);
      drive(-255, -255);
      delay(400);
  } else {
      // **Wall Avoidance Priority**
      if (distanceRight <= minRightDistance) {
          speedLeft = 100;
          speedRight = maxSpeed;
      } else if (distanceLeft <= minLeftDistance) {
          speedLeft = maxSpeed;
          speedRight = 100;
      }
      // **Main Steering & Obstacle Avoidance**
      else if (distanceFront > minFrontDistance) {
          if (distanceRight > 12) {
              speedLeft = maxSpeed - 20;
              speedRight = 100;
          } else if (distanceRight < 7 && distanceLeft > 20) {
              speedLeft = 100;
              speedRight = maxSpeed;
          } else {
              speedLeft = 215;
              speedRight = 255;
          }
      } else if (distanceRight - distanceLeft > 7) {
          speedLeft = 255;
          speedRight = 120;
      } else if (distanceLeft - distanceRight > 7 && distanceRight < 40) {
          speedLeft = 50;
          speedRight = 255;
      }
      
      // **Drive Using PID Speeds**
      drive(speedLeft, speedRight);
  }

  // **Debugging Output**
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" | Right: ");
  Serial.print(distanceRight);
  Serial.print(" | Left: ");
  Serial.print(distanceLeft);
  Serial.print(" | Correction: ");
  Serial.println(correction);

  delay(50);  // **Short delay for better reaction time**
}

void calibrateLineSensor()
{
  
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

  deadzonehigh = result + 50;
  deadzonelow = result - 50;
}

void lineFollower() {

  calibrateLineSensor();
  
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
    if (lineFollowerCount == 1) {
      delay(500);
      stop();
      drive(-255,-255);
      delay(200);
      drive(0,255);
      delay(300);
      mazeLogic();
    }
    else if (lastDirection == -1) {
      drive(255, 0);  // Turn left to search
    } 
    else if (lastDirection == 1) {
      drive(0, 255);  // Turn right to search
    } 
    else {
      drive(214, 255);
    }
  }

  delay(50);
}



void loop() {
  calibrateLineSensor();
  checkBlackDetected();   // Check for black lines
  checkWhiteDetected();   // Check for white lines



  // Example line following logic
  if (blackCount < 5) {
    lineFollower();  // Keep following the line while blackCount is less than 5
  } else {
    drive(-230, 255);  // Move in reverse for a short time if black count exceeds threshold
    delay(300);
    lineFollowerCount++;
    lineFollower();  // Call the line follower again
  }
}

