const int TRANSMIT_PIN = 11;

//define IR Sensor pins
const int SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int NUM_SENSORS = 8; //amount of sensors

int SENSORS_MIN_VALUE = 200;  //Minimum value for Threshold for obstacle avoidance
int SENSORS_MAX_VALUE = 800; //maximum value for obstacle avoidance

int AVG_VALUE = 0;
int DEAD_ZONE_VALUE = 50;

int CALIBRATED_MAX_VALUE = 0;  //lowest value of the line sensor
int CALIBRATED_MIN_VALUE = 1023; //highest value of the line sensor

int READINGS_AMOUNT = 20;

int DEAD_ZONE_W = 0; //temporary value of the deadzone WHITE
int DEAD_ZONE_B = 0; //temporary value of the deadzone BLACK

int LINE_VALUE = 0;
int SURFACE_VALUE = 0;

int DEAD_ZONE_VALUE = 50;

/*
The 8 IR sensors return analog values (0-1023).
Light surfaces (white) give low values (~0-300).
Dark lines (black)  give high values (~700-1023).
*/

void setup() {
  Serial.begin(9600);

  //Set all sensor pins as input
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  pinMode(TRANSMIT_PIN, OUTPUT);
  calibrate();

}

void loop() 
{

  Serial.print("Sensor: ");
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
      int sensorValue = analogRead(SENSOR_PINS[i]);

      if(sensorValue < DEAD_ZONE_B)
      {
        Serial.print("1 ");
      }
      else if (sensorValue > DEAD_ZONE_W)
      {
        Serial.print("0 ");
      }
  }
  delay(100);
  Serial.println();

}

//function to calibrate values
void calibrate()
{
  int sensorValues[NUM_SENSORS];
  
  Serial.println("\nStarting Calibration... Hold still for a few seconds.");
  delay(3000); // Wait to stabilize before calibration

  int readings = READINGS_AMOUNT; // Number of calibration iterations
  int maxReadings[NUM_SENSORS] = {0};
  int minReadings[NUM_SENSORS];
  
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    minReadings[i] = 1023;
  }

  for (int r = 0; r < readings; r++) 
  {
    Serial.print("Reading #");
    Serial.println(r + 1);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = analogRead(SENSOR_PINS[i]);
      if (sensorValues[i] > maxReadings[i]) maxReadings[i] = sensorValues[i];
      if (sensorValues[i] < minReadings[i]) minReadings[i] = sensorValues[i];
    }
    delay(100);
  }

  CALIBRATED_MAX_VALUE = 0;
  CALIBRATED_MIN_VALUE = 1023;
  
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    if (maxReadings[i] > CALIBRATED_MAX_VALUE) CALIBRATED_MAX_VALUE = maxReadings[i];
    if (minReadings[i] < CALIBRATED_MIN_VALUE) CALIBRATED_MIN_VALUE = minReadings[i];
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


