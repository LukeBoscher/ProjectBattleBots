// Motor A1 is pin 10
const int MOTOR_A1 = 10;

// Motor B1 is pin 5
const int MOTOR_A2 = 5;

// Motor B1 is pin 6
const int MOTOR_B1 = 6;

// Motor B2 is pin 9
const int MOTOR_B2 = 9;

const int TRANSMIT_PIN = 11;
  
  int OUT1 = A0;
  int OUT2 = A1;
  int OUT3 = A2;
  int OUT4 = A3;
  int OUT5 = A4;
  int OUT6 = A5;
  int OUT7 = A6;
  int OUT8 = A7;

  int SENSOR1 = 0;
  int SENSOR2 = 0;
  int SENSOR3 = 0;
  int SENSOR4 = 0;
  int SENSOR5 = 0;
  int SENSOR6 = 0;
  int SENSOR7 = 0;
  int SENSOR8 = 0;


  //const int IRPin = 7;
  //const int IRSensor[IRPin]= {1, 2, 3, 4, 5, 6, 7, 8};

  //gray = 02
  // purple = 03
  // blue = 04
  //green = 05
  //yellow = 06
  //orange = 07
  //brown = 08

  //black = ground
  
  //red = VCC

void setup() {
  // put your setup code here, to run once: 
  /*
  Serial.begin(115200);
  for (int i = 0; i < OUT; i++){
  pinMode(OUT[i], INPUT);
  }
*/
  pinMode (OUT1, INPUT);
  pinMode (OUT2, INPUT);
  pinMode (OUT3, INPUT);
  pinMode (OUT4, INPUT);
  pinMode (OUT5, INPUT);
  pinMode (OUT6, INPUT);
  pinMode (OUT7, INPUT);
  pinMode (OUT8, INPUT);

  Serial.begin(115200);

  pinMode(TRANSMIT_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  int sensorStatus = digitalRead([0]);

  if (sensorStatus == HIGH){

    Serial.print("Sensor detected something");
    delay(500);


  } */

  SENSOR1 = analogRead(OUT1);
  SENSOR2 = analogRead(OUT2);
  SENSOR3 = analogRead(OUT3);
  SENSOR4 = analogRead(OUT4);
  SENSOR5 = analogRead(OUT5);
  SENSOR6 = analogRead(OUT6);
  SENSOR7 = analogRead(OUT7);
  SENSOR8 = analogRead(OUT8);

  Serial.println("\nSensor1"); Serial.println(SENSOR1);
  Serial.println("\nSensor2"); Serial.println(SENSOR2);
  Serial.println("\nSensor3"); Serial.println(SENSOR3);
  Serial.println("\nSensor4"); Serial.println(SENSOR4);
  Serial.println("\nSensor5"); Serial.println(SENSOR5);
  Serial.println("\nSensor6"); Serial.println(SENSOR6);
  Serial.println("\nSensor7"); Serial.println(SENSOR7);
  Serial.println("\nSensor8"); Serial.println(SENSOR8);
  delay(1000);

  if (Serial.println = 1)
  {
    maxForward();
  }


//function to make robot go forward
void maxForward() {  
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 255);

  analogWrite(MOTOR_B1, 255);  
  analogWrite(MOTOR_B2, 0);
}

//function to make robot go backwards
void maxBackwards() {
  analogWrite(MOTOR_A1, 255);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 255);
}

//function to make robot go left
void maxLeft(){
  analogWrite(MOTOR_A1, 255);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 255);  
  analogWrite(MOTOR_B2, 0);

}

void dodgeLeft(){
  analogWrite(MOTOR_A1, 191);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 191);  
  analogWrite(MOTOR_B2, 0);

}

//function to make robot go right
void maxRight(){
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 255);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 255);
}

void dodgeRight(){
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 191);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 191);

}

//function to make robot stop
void stopMotor(){
  analogWrite(MOTOR_A1, 0);  
  analogWrite(MOTOR_A2, 0);

  analogWrite(MOTOR_B1, 0);  
  analogWrite(MOTOR_B2, 0);
}
