// Motor A1 is pin 10
const int MOTOR_A1 = 10;
// Motor A2 is pin 5
const int MOTOR_A2 = 5;
// Motor B1 is pin 6
const int MOTOR_B1 = 6;
// Motor B2 is pin 9
const int MOTOR_B2 = 9;

//pin for transmitting data
const int TRANSMIT_PIN = 11;

//maxspeed for motor
const int MAX_SPEED = 240;

//speed for dodging
const int DODGE_SPEED = 191;


//define IR Sensor pins
const int SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int NUM_SENSORS = 8; //amount of sensors

int THRESHOLD_MIN = 200;  //Minimum value for Threshold for obstacle avoidance
int THRESHOLD_MAX = 800; //maximum value for obstacle avoidance

int SENSORS_MIN_VALUE = 200;  //Minimum value for Threshold for obstacle avoidance
int SENSORS_MAX_VALUE = 800; //maximum value for obstacle avoidance

int AVG_VALUE = 0;

int CALIBRATED_MAX_VALUE = 0;  //lowest value of the line sensor
int CALIBRATED_MIN_VALUE = 1023; //highest value of the line sensor

int DEAD_ZONE_W = 0; //temporary value of the deadzone WHITE
int DEAD_ZONE_B = 0; //temporary value of the deadzone BLACK

//number of times function calibrate calibrates
int READINGS_AMOUNT = 20;

//threshold value for deadzones
int DEAD_ZONE_VALUE = 50;

//stores line and surface values
int LINE_VALUE = 0;
int SURFACE_VALUE = 0;


/*
The 8 IR sensors return analog values (0-1023).
Dark lines (black) give low values (~0-300).
Light surfaces (white) give high values (~700-1023).

for some reason it backwards
*/

void setup() {
  Serial.begin(9600);


  //Set all sensor pins as input
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  //set pins for output
  pinMode(TRANSMIT_PIN, OUTPUT);

  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  calibrate(); //calibrates sensors
}

void loop() {

  bool obstacleDetected = false;  
  int sensorValues[NUM_SENSORS];
  int position = 0;
  int total = 0, error = 0;

  Serial.print("Sensors: ");
  //read all values from the sesnos and calculate tota;
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = analogRead(SENSOR_PINS[i]); //read all IRsensor data
      position += sensorValues[i] * i; //calculate position weight
      total += sensorValues[i]; //calculate total position value


    Serial.print(sensorValues[i]); //print IRsensor datas
    Serial.print(" ");

    //checks if sensors detect any obstacles
    if(sensorValues[i] < DEAD_ZONE_B)
    {
      obstacleDetected = true;
    }

  }

  // calculates error for line follow
   error = (total > 0) ? (position / total) - (NUM_SENSORS / 2) : 0;

  Serial.println();
  Serial.println("---------------------------------");
  delay(200);

  //debug print if obstacle detected or not
  if (!obstacleDetected) 
  {
  Serial.println("NO Obstacle Detected");
  adjustMovement(error);
  } 
  else 
  {
    Serial.println("Obstacle Detected");
    avoidObstacle(sensorValues);
  }


  int LEFT_SIDE = sensorValues[0] + sensorValues[1]; //reads the left most sensor 
  int RIGHT_SIDE = sensorValues[7] + sensorValues[6]; //reads the right most senor
  int CENTER = sensorValues[3] + sensorValues[4]; // reads the middle most sensor
  bool MIDDLE_LINE = (sensorValues[3] < DEAD_ZONE_B) && (sensorValues[4] < DEAD_ZONE_B);

/*
  // if no obstacle detected adjust left or right motor speeds speeds 
  if (MIDDLE_LINE) {


    //motor moves forward based on sensor 
  } 
  else 
  {
    // Obstacle avoidance logic
    if 
    (sensorValues[0] + sensorValues[1] > sensorValues[6] + sensorValues[7]) 
    {
      slowTurnRight(); //If obstacle is left turn right
    } 
    else 
    {
      slowTurnLeft(); // If obstacle is right turn left
    }
  }

  delay(10);
*/

}

void adjustMovement(int error) 
{
    int speedLeft = MAX_SPEED - (error * 5);
    int speedRight = MAX_SPEED + (error * 5);
    speedLeft = constrain(speedLeft, 0, MAX_SPEED);
    speedRight = constrain(speedRight, 0, MAX_SPEED);
    drive(speedLeft, speedRight);
}

void avoidObstacle(int sensorValues[]) 
{
    int LEFT_SIDE = sensorValues[0] + sensorValues[1];
    int RIGHT_SIDE = sensorValues[6] + sensorValues[7];
    if (LEFT_SIDE > RIGHT_SIDE) {
        slowTurnRight();
    } else {
        slowTurnLeft();
    }
}

void drive(int speedLeft, int speedRight) 
{
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, speedLeft);
    analogWrite(MOTOR_B1, speedRight);
    analogWrite(MOTOR_B2, 0);
}


void turnLeft()
{

  analogWrite(MOTOR_A1, 150);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 150);  
  analogWrite(MOTOR_B2, 0);
}

void turnRight()
{
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 150);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 150);
}


//function to make robot go forward
void moveForward() {  
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, MAX_SPEED);

  analogWrite(MOTOR_B1, MAX_SPEED);  
  analogWrite(MOTOR_B2, 0);
}

//function to make robot go backwards
void moveBackwards() {
  analogWrite(MOTOR_A1, MAX_SPEED);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, MAX_SPEED);
}

//function to make robot go left
void moveLeft(){
  analogWrite(MOTOR_A1, MAX_SPEED);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, MAX_SPEED);  
  analogWrite(MOTOR_B2, 0);

}

void dodgeLeft(){
  analogWrite(MOTOR_A1, DODGE_SPEED);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, DODGE_SPEED);  
  analogWrite(MOTOR_B2, 0);

}

//function to make robot go right
void moveRight(){
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, MAX_SPEED);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, MAX_SPEED);
}

void dodgeRight(){
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, DODGE_SPEED);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, DODGE_SPEED);

}

//function to make robot stop
void stopMotor(){
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 0);
}

void slowTurnRight()
{
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 125);  
  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 125);
}

void slowTurnLeft()
{
  analogWrite(MOTOR_A1, 150);  
  analogWrite(MOTOR_A2, 0);  
  analogWrite(MOTOR_B1, 150);  
  analogWrite(MOTOR_B2, 0);
}

//function to calibrate sensor readings
void calibrate()
{
  stopMotor();
  int sensorValues[NUM_SENSORS];
  
  Serial.println("\nStarting Calibration this takes a few seconds.");
  delay(3000); // Wait to stabilize before calibration

  int readings = READINGS_AMOUNT; // Number of calibration iterations
  int maxReadings[NUM_SENSORS] = {0};
  int minReadings[NUM_SENSORS];
  
  //min readings
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    minReadings[i] = 1023;
  }

//collects the multiple reads
  for (int r = 0; r < readings; r++) 
  {
    Serial.print("Reading #");
    Serial.println(r + 1);
    
    //read sensor values
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = analogRead(SENSOR_PINS[i]);
      if (sensorValues[i] > maxReadings[i]) maxReadings[i] = sensorValues[i];
      if (sensorValues[i] < minReadings[i]) minReadings[i] = sensorValues[i];
    }
    delay(100);
  }

  //final calibrated values
  CALIBRATED_MAX_VALUE = 0;
  CALIBRATED_MIN_VALUE = 1023;
  
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    if (maxReadings[i] > CALIBRATED_MAX_VALUE) CALIBRATED_MAX_VALUE = maxReadings[i]; //max value
    if (minReadings[i] < CALIBRATED_MIN_VALUE) CALIBRATED_MIN_VALUE = minReadings[i]; //min value
  }
  
  AVG_VALUE = (CALIBRATED_MAX_VALUE + CALIBRATED_MIN_VALUE) / 2;
  DEAD_ZONE_W = AVG_VALUE - DEAD_ZONE_VALUE;
  DEAD_ZONE_B = AVG_VALUE + DEAD_ZONE_VALUE;

  LINE_VALUE = CALIBRATED_MIN_VALUE;
  SURFACE_VALUE = CALIBRATED_MAX_VALUE;

  Serial.println("Calibration Complete:");

  Serial.print("Calibrated max value (Surface): "); Serial.println(CALIBRATED_MAX_VALUE);

  Serial.print("Calibrated min value (Line): "); Serial.println(CALIBRATED_MIN_VALUE);

  Serial.print("Average value: "); Serial.println(AVG_VALUE);

  Serial.print("Dead zone white: "); Serial.println(DEAD_ZONE_W);

  Serial.print("Dead zone black: "); Serial.println(DEAD_ZONE_B);
}

