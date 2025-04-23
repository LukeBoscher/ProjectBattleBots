/*
* ==========================================================
*  Project: Battlebots
*  Board  : Arduino Nano
*  Author : Robbin & Tobias Schipper
*  Version: 1.0
*  
*
* ==========================================================
*  PINOUT ARDUINO NANO
*  ---------------------------------------------------------
*  | Pin | Functie      | Opmerking                        |
*  |-----|-------------|----------------------------------|
*  | D0  | RX          | Seriële communicatie (in)       |
*  | D1  | TX          | Seriële communicatie (uit)      |
*  | D2  | Digitaal I/O| vrij                            |
*  | D3  | Digitaal I/O| vrij                            |
*  | D4  | Digitaal I/O| vrij                            |
*  | D5  | Digitaal I/O| PWM motor rechts B1             |
*  | D6  | Digitaal I/O| PWM motor links A1              |
*  | D7  | Digitaal I/O| vrij                            |
*  | D8  | Digitaal I/O| IfrSensor Trig                  |
*  | D9  | Digitaal I/O| IfrSensor Echo                  |
*  | D10 | Digitaal I/O| PWM motor rechts B2             |
*  | D11 | Digitaal I/O| PWM motor links A2              |
*  | D12 | Digitaal I/O| input Gripper                   |
*  | D13 | Digitaal I/O| Data pin voor Neopixels         |
*  | A0  | Analoog In  | LijnSensor 1 Rechts             |
*  | A1  | Analoog In  | LijnSensor 2                    |
*  | A2  | Analoog In  | LijnSensor 3                    |
*  | A3  | Analoog In  | LijnSensor 4                    |
*  | A4  | Analoog In  | LijnSensor 5                    |
*  | A5  | Analoog In  | LijnSensor 6                    |
*  | A6  | Analoog In  | LijnSensor 7                    |
*  | A7  | Analoog In  | LijnSensor 8 Links              |
*  -------------------------------------------------------
*/

//-------------LIBRARIES-----------------------------------------
#include <Adafruit_NeoPixel.h>

//-------------NEO PIXEL SETUP------------------------------------
#define PIN 13       // Data pin connected to DIN
#define NUM_PIXELS 4 // neopixel led count
Adafruit_NeoPixel strip(NUM_PIXELS, PIN, NEO_RGB + NEO_KHZ800);

//-------------PIN SETUP------------------------------------------
#define BUTTON_PIN_1 2
bool _buttonState1 = 0;

#define BUTTON_PIN_2 3
bool _buttonState2 = 0;

//-------------Links motor
#define MOTOR_L1 6
#define MOTOR_L2 11

//-------------Rechts motor
#define MOTOR_R1 10
#define MOTOR_R2 5

//-------------IFR Sensor
#define TRIG_PIN 8
#define ECHO_PIN 9
float _duration, _distance;

//-------------Gripper
#define GRIPPER_PIN 12

//-----------lijnsensor
#define NUM_SENSORS 8
const int _lineSensor[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};
int _sensorReadings[NUM_SENSORS];
int _deadZoneLow = 0;
int _deadZoneHigh = 0;
int _lastDistance = 0;

//-----------start & end variabelen
unsigned long _startTime = 0; // Tijdstip waarop de sensor voor het eerst iets detecteert
bool _timerRunning = false;
bool _start = false; 

//-----------DEBUGGING
//#define SENSOR_VALUE

//-----------MOTOR SPEEDS
#define FULL_SPEED 255
#define STEADY_SPEED 214
#define SLOW_SPEED 180
#define SLOWER_SPEED 60
#define SLOWEST_SPEED 10
#define SEARCH_SPEED 50

//-----------LINEFOLLOW LOGIC
int _lastDirection = 0;  // 0 = rechtdoor, -1 = links, 1 = rechts
bool _isTurning = false;  // Houdt bij of de robot aan het draaien is
bool _lastTurnRight = true; // Houdt bij of de laatste draai naar rechts was
unsigned long _turnStartTime = 0; // Stores the start time of the turn
const unsigned long SHARP_TURN_DURATION = 400; // Duration for sharp turns in milliseconds

//-------------SETUP
void setup() {
    Serial.begin(9600);

    //motor setup
    pinMode(MOTOR_L1, OUTPUT);
    pinMode(MOTOR_L2, OUTPUT);
    pinMode(MOTOR_R1, OUTPUT);
    pinMode(MOTOR_R2, OUTPUT); 
    //IFR Setup
    pinMode(TRIG_PIN, OUTPUT);  
    pinMode(ECHO_PIN, INPUT); 
    //gripper setup
    pinMode(GRIPPER_PIN, OUTPUT); 

    //output op uit zetten
    digitalWrite(MOTOR_L1, HIGH);
    digitalWrite(MOTOR_L2, HIGH);
    digitalWrite(MOTOR_R1, HIGH);
    digitalWrite(MOTOR_R2, HIGH);
    digitalWrite(TRIG_PIN, HIGH);
    digitalWrite(GRIPPER_PIN, HIGH);

    // Zet alle sensors als input
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(_lineSensor[i], INPUT);
    }

    pinMode(BUTTON_PIN_1, INPUT);
    pinMode(BUTTON_PIN_2, INPUT);

    //Pixel setup
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}


//-------------LOOP
void loop() {
    _buttonState1 = digitalRead(BUTTON_PIN_1);
    _buttonState2 = digitalRead(BUTTON_PIN_2);

    if (_buttonState1 == LOW) {
        GripperOpen();
    } else if (_buttonState2 == LOW) {
        GripperClosed();
    }
    MazeLine();
}

//--------------------BEWEGINGSFUNCTIES
//-------------MOTOR CONTROL
void MotorControl(int leftForward, int leftBackward, int rightForward, int rightBackward) {
    // Left motor control
    analogWrite(MOTOR_L1, leftBackward); // Backward motion
    analogWrite(MOTOR_L2, leftForward); // Forward motion

    // Right motor control
    analogWrite(MOTOR_R1, rightBackward); // Backward motion
    analogWrite(MOTOR_R2, rightForward); // Forward motion
}

void StopMotorControl() { 
    BrakeLight();
    analogWrite(MOTOR_L1, 0); 
    analogWrite(MOTOR_L2, 0); 
    analogWrite(MOTOR_R1, 0); 
    analogWrite(MOTOR_R2, 0); 
} 

//-------------GRIPPER FUNCTIE
void Gripper(int pulse) {
    static unsigned long timer;
    static int lastPulse;
    if (millis() > timer) {
        if (pulse > 0) {
            lastPulse = pulse;
        } else {
            pulse = lastPulse;
        }
        digitalWrite(GRIPPER_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER_PIN, LOW);
        timer = millis() + 20; //20ms interval voor servo
    }
}

//-------------GRIPPER OPEN
void GripperOpen() {
    Gripper(1900);
}

//-------------GRIPPER SLUITEN
void GripperClosed() {
    Gripper(1250);
}

//-------------ON/OFF IFRSENSOR
void IfrSensor() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
}

//-------------HAAL AFSTAND OP VAN IFRSENSOR
void IfrInformation() {
    IfrSensor();
    _duration = pulseIn(ECHO_PIN, HIGH);
    _distance = (_duration * .0343) / 2;
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 100) { // 100 millisecond delay
        lastMillis = millis();
    }
}

//-------------Maze Line Follower
void MazeLine() {
    int _sensorReadings[NUM_SENSORS]; 
    int sum = 0; 
    
    for (int i = 0; i < NUM_SENSORS; i++) { 
        _sensorReadings[i] = analogRead(_lineSensor[i]);
        sum += _sensorReadings[i]; 
    }
    
    int average = sum / NUM_SENSORS;
    _deadZoneLow = average - 50; // Increased range for smoother transitions
    _deadZoneHigh = average + 100;
    int currentDirection = 0;

    // === VLOEIENDE BOCHTEN (RECHTS PRIORITEIT) ===
    //Checken of er lijn naar rechts is
    if (_sensorReadings[7] >= _deadZoneHigh || _sensorReadings[6] >= _deadZoneHigh) { 
        // Sterke bocht naar rechts
        MotorControl(STEADY_SPEED, 0, 0, STEADY_SPEED); 
        currentDirection = 1;
    } 
    //Checken of er lijn naar links is
    else if (_sensorReadings[1] >= _deadZoneHigh || _sensorReadings[0] >= _deadZoneHigh) { 
        // Sterke bocht naar links
        MotorControl(0, STEADY_SPEED, STEADY_SPEED, 0); 
        currentDirection = -1;
    }
    //Als er geen bochten beschikbaar is en er een lijn is rechtdoor rijden
    else if (_sensorReadings[4] >= _deadZoneHigh && _sensorReadings[3] >= _deadZoneHigh) { 
        // Rechtdoor rijden
        MotorControl(STEADY_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 0;  
    }  
    //Lijn rechts lichte correctie naar links
    else if (_sensorReadings[2] >= _deadZoneHigh) { 
        MotorControl(SLOW_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 5;
    } 
    else if (_sensorReadings[1] >= _deadZoneHigh) { 
        MotorControl(SLOW_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 6;
    }
    //Veel lijn links lichte correctie naar rechts
    else if (_sensorReadings[5] >= _deadZoneHigh) { 
        MotorControl(STEADY_SPEED, 0, SLOW_SPEED, 0);  
        currentDirection = 3;  
    } 
    // Te veel lijn links sterkere correctie naar rechts
    else if (_sensorReadings[6] >= _deadZoneHigh) { 
        MotorControl(STEADY_SPEED, 0, SLOWER_SPEED, 0);  
        currentDirection = 4;  
    } 
    // Alles lijnen kwijt 180 graden draaien
    else if (sum <= _deadZoneLow * NUM_SENSORS) { 
        // Geen lijn meer zichtbaar → dead-end
        currentDirection = 7;
        MotorControl(0, SLOW_SPEED, SLOW_SPEED, 0); 
    } else {  
        // Geen sensor triggert een sterke reactie → rechtdoor blijven rijden
        MotorControl(STEADY_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 0;  
    }  

    // Debugging output
    switch (currentDirection) {
        case -1: Serial.println("Bocht naar links -> draai links!"); break;
        case 0: Serial.println("Midden op lijn -> rechtdoor!"); break;
        case 1: Serial.println("Bocht naar rechts -> draai rechts!"); break;
        case 3: Serial.println("Iets rechts -> stuur beetje bij!"); break;
        case 4: Serial.println("Meer rechts -> stuur sterker bij!"); break;
        case 5: Serial.println("Iets links -> stuur beetje bij!"); break;
        case 6: Serial.println("Meer lijn links -> stuur sterker bij!"); break;
        case 7: Serial.println("Doodlopend lijn -> rechts draaien...."); break;
        case 8: Serial.println("T-split gedetecteerd!"); break;
    }

    // Update lights based on direction
    switch (currentDirection) {
        case -1: BlinkerLeft(); break;
        case 0: RegularLight(); break;
        case 1: BlinkerRight(); break;
        case 2: RegularLight(); break;
        case 3: RegularLight(); break;
        case 4: BlinkerLeft(); break;
        case 5: RegularLight(); break;
        case 6: BlinkerLeft(); break;
        case 7: BlinkerRight(); break;
        case 8: RegularLight(); break;
        default: RegularLight(); break;
    }

    delay(75);
}

void ConeDrop() {
    unsigned long turnStartTime = 0;
    bool turning = false;

    if (_distance > 30 && _distance < 50) {
        GripperOpen();
        MotorControl(STEADY_SPEED, 0, STEADY_SPEED, 0);
        RegularLight(); // Turn on regular light while moving forward

        if (_distanceE < 5) {
            GripperClosed();
            _start = true;
            BrakeLight(); // Turn on brake light when stopping
        }

        if (!turning) {
            turnStartTime = millis();
            turning = true;
        }

        if (turning && millis() - turnStartTime >= 275) {
            MotorControl(0, SLOW_SPEED, SLOW_SPEED, 0); // Turn left
            BlinkerLeft(); // Turn on left blinker while turning
        }

        if (turning && millis() - turnStartTime >= 1000) {
            MotorControl(STEADY_SPEED, 0, STEADY_SPEED, 0); // Move forward
            RegularLight(); // Switch back to regular light
            turning = false;
        }
    }

    if (_start) {
        if (sensorReadings[7] >= _deadZoneHigh && sensorReadings[0] >= _deadZoneHigh) {
            if (!_timerRunning) {
                _startTime = millis(); // Start the timer
                _timerRunning = true;
            }
        } else {
            _timerRunning = false; // Reset the timer if sensors detect nothing
            _startTime = 0;
        }

        if (_timerRunning && millis() - _startTime >= 250) {
            GripperOpen();
            MotorControl(-STEADY_SPEED, 0, -STEADY_SPEED, 0); // Move backward
            BrakeLight(); // Turn on brake light while reversing
            _timerRunning = false; // Reset the timer after the action
        }

        MazeLine(); // Start the MazeLine function if _START is true
    }
}

//-------------LIGHT FUNCTIONS------------------------------------

//-------------DEFAULT INDICATOR PROGRAM
void Blinkers(int upper, int lower, bool active) {
    static unsigned long previousMillis = 0;
    static bool ledState = false;  // Ensure ledState is retained across calls
    unsigned long currentMillis = millis();

    if (active) {  // Only blink if "active" is true
        if (currentMillis - previousMillis >= 250) {  
            previousMillis = currentMillis;
            ledState = !ledState;  // Toggle state
        }
    } else {
        ledState = false;
        previousMillis = 0;  // Reset timer if blinking stops
    }

    if (ledState) {
        strip.setPixelColor(upper, strip.Color(255, 69, 0));  // Orange ON
        strip.setPixelColor(lower, strip.Color(255, 69, 0));  // Orange ON
    } else {
        strip.setPixelColor(upper, strip.Color(100, 0, 0));  // Red
        strip.setPixelColor(lower, strip.Color(100, 100, 100)); // White
    }

    strip.show();  // Update the strip to reflect changes
}

//-------------LEFT BLINKER
void BlinkerLeft() {
    strip.clear();
    Blinkers(0, 3, true);
    strip.setPixelColor(1, strip.Color(150, 0, 0));  // Red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // White
    strip.show();
}

//-------------RIGHT BLINKER
void BlinkerRight() {
    strip.clear();
    Blinkers(1, 2, true);
    strip.setPixelColor(0, strip.Color(150, 0, 0));  // Red
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // White
    strip.show();
}

//-------------BRAKE LIGHT
void BrakeLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(255, 0, 0));  // Red
    strip.setPixelColor(1, strip.Color(255, 0, 0));  // Red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // White
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // White
    strip.show();
}

//-------------DEFAULT LIGHT
void RegularLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(50, 0, 0));  // Red
    strip.setPixelColor(1, strip.Color(50, 0, 0));  // Red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // White
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // White
    strip.show();
}
