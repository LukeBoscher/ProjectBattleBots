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
const float minFrontDistance = 15.0; // If closer than this, stop or reverse

// **PID Function for Steering Correction**
int PIDControl(float error) {
  integral += error;
  float derivative = error - lastError;
  lastError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void loop() {
  float distanceFront = measureDistance("voor");
  float distanceLeft = measureDistance("links");
  float distanceRight = measureDistance("rechts");

  // **Calculate PID Error for Steering (Right Preference)**
  float error = (distanceRight - distanceLeft) + 5;  // Encourages right turns
  int correction = PIDControl(error);

  // **Calculate Wheel Speeds**
  int speedLeft = baseSpeed - correction;
  int speedRight = baseSpeed + correction;

  // **Keep Speeds Within Limits**
  speedLeft = constrain(speedLeft, minSpeed, maxSpeed);
  speedRight = constrain(speedRight, minSpeed, maxSpeed);

  // **Wall Avoidance Priority**
  if (distanceRight < minRightDistance) {
    // **Too close to right wall! Strong left correction**
    speedLeft = 100;
    speedRight = maxSpeed;
  } 
  if (distanceFront < minFrontDistance)
  {
    speedLeft = -255;
    speedRight = -255;
  }

  // **Main Steering & Obstacle Avoidance**
  else if (distanceFront > minFrontDistance) {
    if (distanceRight > 11) {
      speedLeft = maxSpeed;   // Turn right
      speedRight = 100;
    } 
    else if (distanceRight < 7 && distanceLeft > 11) {
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
  else if (distanceFront <= minFrontDistance && distanceLeft <= 15 && distanceRight <= 15) {
    if (distanceLeft > distanceRight) {
      drive(255, -255);  // Reverse and turn left
    } else {
      drive(-255, 255);  // Reverse and turn right
    }
    delay(400);

    drive(-255, 255);  
    delay(500);

    drive(-255, -255);
    delay(100);
  }

  // **Drive Using PID Speeds**
  drive(speedLeft, speedRight);

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
