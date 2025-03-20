// Motor A1 is pin 10
const int a1 = 10;

// Motor B1 is pin 5
const int a2 = 5;

// Motor B1 is pin 6
const int b1 = 6;

// Motor B2 is pin 9
const int b2 = 9;

const int maxSpeed = 255; //MAX_SPEED
const int turnSpeed = 240;
const int midSpeed = 127;

void setup() {
  //motor A have output
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);

  //motor B have output
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
}

void loop() {

  maxForward(); //starts function forward
  delay(1000); //wait for 1 second

  stopMotor(); //start function stop
  delay(1000); //wait for 5 seconds

  maxBackwards(); //starts function backwards
  delay(1000); //wait for 1 second

  stopMotor(); //start function stop
  delay(1000); //wait for 5 seconds

  maxLeft(); //starts function left
  delay(500); //wait for 1 second

  stopMotor(); //start function stop
  delay(1000); //wait for 5 seconds

  maxRight(); //starts function right
  delay(500); //wait for 1 second

  stopMotor(); //start function stop
  delay(2000); //wait for 5 seconds
  

}

void maxForward() {  
  analogWrite(a1, 0);  
  analogWrite(a2, maxSpeed);

  analogWrite(b1, maxSpeed);  
  analogWrite(b2, 0);

  delay(3000);
}


//function to make robot go backwards
void maxBackwards() {
  analogWrite(a1, maxSpeed);  
  analogWrite(a2, 0);

  analogWrite(b1, 0);  
  analogWrite(b2, maxSpeed);
}

//function to make robot go left
void maxLeft(){
  analogWrite(a1, turnSpeed);  
  analogWrite(a2, 0);

  analogWrite(b1, turnSpeed);  
  analogWrite(b2, 0);


}

//function to make robot go right
void maxRight(){
  analogWrite(a1, 0);  
  analogWrite(a2, turnSpeed);

  analogWrite(b1, 0);  
  analogWrite(b2, turnSpeed);
}

//function to make robot stop
void stopMotor(){
  analogWrite(a1, 0);  
  analogWrite(a2, 0);

  analogWrite(b1, 0);  
  analogWrite(b2, 0);
}

//function to make robot go backwards
void midBackwards() {
  analogWrite(a1, midSpeed);  
  analogWrite(a2, 0);

  analogWrite(b1, 0);  
  analogWrite(b2, midSpeed);
}

//function to make robot go left
void midLeft(){
  analogWrite(a1, midSpeed);  
  analogWrite(a2, 0);

  analogWrite(b1, midSpeed);  
  analogWrite(b2, 0);


}

//function to make robot go right
void midRight(){
  analogWrite(a1, 0);  
  analogWrite(a2, midSpeed);

  analogWrite(b1, 0);  
  analogWrite(b2, midSpeed);
}




