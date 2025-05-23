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

// Light sensors
int lichtsensorWaarden[8];
const int lichtsensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };

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
  for (int i = 0; i < 8; i++) {
    pinMode(lichtsensorPins[i], INPUT);
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
  for (int i = 0; i < 8; i++) {
    lichtsensorWaarden[i] = analogRead(lichtsensorPins[i]);
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

int firstOptimisticCorner = 0;


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
const float minLeftDistance = 6.0;   // If closer than this, must turn right
const float minFrontDistance = 15.0; // If closer than this, stop or reverse
const float deadEndThreshold = 5.0;  // Threshold to detect if it's completely surrounded

// **PID Function for Steering Correction**
int PIDControl(float error) {
  integral += error;
  float derivative = error - lastError;
  lastError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

float prevDistanceFront = 0;

void loop() {
  float distanceFront = measureDistance("voor");
  float distanceLeft = measureDistance("links");
  float distanceRight = measureDistance("rechts");

  // **Calculate PID Error for Steering (Right Preference)**
  float error = (distanceRight - distanceLeft) + 2;  // Encourages right turns
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
    drive(-255,-255);
    delay(400);
  } else {
      // **Wall Avoidance Priority**
    if (distanceRight <= minRightDistance) {
      // **Too close to right wall! Strong left correction**
      speedLeft = 100;
      speedRight = maxSpeed;
    } 
    else if (distanceLeft <= minLeftDistance) {
      // **Too close to left wall! Strong right correction**
      speedLeft = maxSpeed;
      speedRight = 100;
    }
    // **Main Steering & Obstacle Avoidance**
    else if (distanceFront > minFrontDistance) {
      if (distanceRight > 12) {
        speedLeft = maxSpeed-20;   // Turn right
        speedRight = 100;
      } 
      else if (distanceRight < 7 && distanceLeft > 20) {
        speedLeft = 100;
        speedRight = maxSpeed;  // Turn left
      } 
      else {
        speedLeft = 215;
        speedRight = 255;  // Slight right bias
      }
    } 
    else if (distanceRight - distanceLeft > 7) {
      speedLeft = 255;  
      speedRight = 120;   // Turn right slightly
    } 
    else if (distanceLeft - distanceRight > 7 && distanceRight < 40) {
      speedLeft = 50;  
      speedRight = 255;  // Turn left slightly
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
