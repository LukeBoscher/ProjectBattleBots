// Motor A1 is pin 10
const int a1 = 10;

// Motor B1 is pin 5
const int a2 = 5;

// Motor B1 is pin 6
const int b1 = 6;

// Motor B2 is pin 9
const int b2 = 9;

//yellow wire is trigger
const int trigPin = 13;

//green wire is echo 
const int echoPin = 12; //ECHO_PIN

//trigger distance of obstacle in cm
const int obstacleThreshold = 10;  

//red wire = 5V
//black wire = ground

const int MAX_SPEED = 240;

const int TURN_SPEED= 240;


const int dodgeSpeed = 191; //191

void setup() {
  // initialize serial communication:
  Serial.begin(115200);

  //motor A have output
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);

  //motor B have output
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  //input echo
  pinMode(echoPin, INPUT);
  //output trigger
  pinMode(trigPin, OUTPUT);

}

void loop() {

  dodge ();
  
}

void dodge (){
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH, 20000);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);


  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(100);

  if (duration == 0) 
  {
  Serial.println("No echo received");
  return;
  }

  maxForward();


  if (cm <= obstacleThreshold){
    stopMotor();
    delay(300); // When robot sees obstacle robot motor stops

    maxBackwards(); //robot goes backwards
    delay(300);

    maxLeft(); // robot turns left
    delay(450);

    maxForward(); //robot goes backwards
    delay(500);

    maxRight(); // robot turns right
    delay(500);

    maxForward(); //robot goes backwards
    delay(1500);

    maxRight(); // robot turns left
    delay(500);

    maxForward(); // robot moves forward again
    delay(500);

    maxLeft(); // robot turns left
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

//function to make robot go forward
void maxForward() {  
  analogWrite(a1, 0);  
  analogWrite(a2, MAX_SPEED);

  analogWrite(b1, MAX_SPEED);  
  analogWrite(b2, 0);
}

//function to make robot go backwards
void maxBackwards() {
  analogWrite(a1, MAX_SPEED);  
  analogWrite(a2, 0);

  analogWrite(b1, 0);  
  analogWrite(b2, MAX_SPEED);
}

//function to make robot go left
void maxLeft(){
  analogWrite(a1, TURN_SPEED);  
  analogWrite(a2, 0);

  analogWrite(b1, TURN_SPEED);  
  analogWrite(b2, 0);

}

void dodgeLeft(){
  analogWrite(a1, dodgeSpeed);  
  analogWrite(a2, 0);

  analogWrite(b1, dodgeSpeed);  
  analogWrite(b2, 0);

}

//function to make robot go right
void maxRight(){
  analogWrite(a1, 0);  
  analogWrite(a2, TURN_SPEED);

  analogWrite(b1, 0);  
  analogWrite(b2, TURN_SPEED);
}

void dodgeRight(){
  analogWrite(a1, 0);  
  analogWrite(a2, dodgeSpeed);

  analogWrite(b1, 0);  
  analogWrite(b2, dodgeSpeed);
}

//function to make robot stop
void stopMotor(){
  analogWrite(a1, 0);  
  analogWrite(a2, 0);

  analogWrite(b1, 0);  
  analogWrite(b2, 0);
}

