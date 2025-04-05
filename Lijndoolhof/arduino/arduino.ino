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
#define NUMPIXELS 4  // neopixel led count
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

//-------------PIN SETUP------------------------------------------
//-------------Links motor
#define motor_L1 6
#define motor_L2 11

//-------------Rechts motor
#define motor_R1 10
#define motor_R2 5

//-------------IFR Sensor
#define TRIGPIN 8
#define ECHOPIN 9
float _DURATION, _DISTANCE;

//-------------Gripper
#define GRIPPER 12

//-----------lijnsensor
#define _NUM_SENSORS 8
const int LineSensor[_NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};
int sensorReadings[_NUM_SENSORS];
int _DEADZONELOW = 0;
int _DEADZONEHIGH = 0;
int _LASTDISTANCE = 0;

//-----------start & end variabelen
unsigned long _STARTTIME = 0; // Tijdstip waarop de sensor voor het eerst iets detecteert
bool _TIMERRUNNING = false;
bool _START = false; 

//-----------DEBUGGING
//#define SENSORVALUE

//-----------MOTOR SPEEDS
#define FULLSPEED 255
#define STEADY_SPEED 214
#define SLOW_SPEED 180
#define SLOWER_SPEED 60
#define SLOWEST_SPEED 10
#define SEARCH_SPEED 50

//-----------LINEFOLLOW LOGIC
int _lastDirection = 0;  // 0 = rechtdoor, -1 = links, 1 = rechts
bool isTurning = false;  // Houdt bij of de robot aan het draaien is
bool lastTurnRight = true; // Houdt bij of de laatste draai naar rechts was
unsigned long turnStartTime = 0; // Stores the start time of the turn
const unsigned long sharpTurnDuration = 400; // Duration for sharp turns in milliseconds

//-------------SETUP
void setup() {
    Serial.begin(9600);

    //motor setup
    pinMode(motor_L1, OUTPUT);
    pinMode(motor_L2, OUTPUT);
    pinMode(motor_R1, OUTPUT);
    pinMode(motor_R2, OUTPUT); 
    //IFR Setup
    pinMode(TRIGPIN, OUTPUT);  
    pinMode(ECHOPIN, INPUT); 
    //gripper setup
    pinMode(GRIPPER, OUTPUT); 

    //output op uit zetten
    digitalWrite(motor_L1, HIGH);
    digitalWrite(motor_L2, HIGH);
    digitalWrite(motor_R1, HIGH);
    digitalWrite(motor_R2, HIGH);
    digitalWrite(TRIGPIN, HIGH);
    digitalWrite(GRIPPER, HIGH);

    // Zet alle sensors als input
    for (int i = 0; i < _NUM_SENSORS; i++) {
        pinMode(LineSensor[i], INPUT);
    }

    //Pixel setup
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}


//-------------LOOP
void loop() {
    mazeLine();
}

//--------------------BEWEGINGSFUNCTIES
//-------------MOTOR CONTROL
void motorControl(int leftForward, int leftBackward, int rightForward, int rightBackward) {
    // Left motor control
    analogWrite(motor_L1, leftBackward); // Backward motion
    analogWrite(motor_L2, leftForward); // Forward motion

    // Right motor control
    analogWrite(motor_R1, rightBackward); // Backward motion
    analogWrite(motor_R2, rightForward); // Forward motion
}

void stopMotorControl() { 
    brakeLight();
    analogWrite(motor_L1, 0); 
    analogWrite(motor_L2, 0); 
    analogWrite(motor_R1, 0); 
    analogWrite(motor_R2, 0); 
} 

//-------------GRIPPER FUNCTIE
void gripper(int pulse) {
    static unsigned long timer;
    static int lastPulse;
    if (millis() > timer) {
        if (pulse > 0) {
            lastPulse = pulse;
        } else {
            pulse = lastPulse;
        }
        digitalWrite(GRIPPER, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER, LOW);
        timer = millis() + 20; //20ms interval voor servo
    }
}

//-------------GRIPPER OPEN
void gripperOpen() {
    gripper(1800);
}

//-------------GRIPPER SLUITEN
void gripperClosed() {
    gripper(1050);
}

//-------------ON/OFF IFRSENSOR
void ifrSensor() {
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);
}

//-------------HAAL AFSTAND OP VAN IFRSENSOR
void ifrInformation() {
    ifrSensor();
    _DURATION = pulseIn(ECHOPIN, HIGH);
    _DISTANCE = (_DURATION*.0343)/2;
    // Serial.print("Distance: ");
    // Serial.println(_DISTANCE);
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 100) { // 100 millisecond delay
        lastMillis = millis();
    }
}

//-------------Maze Line Follower
void mazeLine() {
    int sensorReadings[_NUM_SENSORS]; 
    int sum = 0; 
    
    for (int i = 0; i < _NUM_SENSORS; i++) { 
        sensorReadings[i] = analogRead(LineSensor[i]);
        sum += sensorReadings[i]; 
    }
    
    int average = sum / _NUM_SENSORS;
    _DEADZONELOW = average - 50; // Increased range for smoother transitions
    _DEADZONEHIGH = average + 100;
    int currentDirection = 0;

    // === VLOEIENDE BOCHTEN (RECHTS PRIORITEIT) ===
    //Checken of er lijn naar rechts is
    if (sensorReadings[7] >= _DEADZONEHIGH || sensorReadings[6] >= _DEADZONEHIGH) { 
        // Sterke bocht naar rechts
        motorControl(STEADY_SPEED, 0, 0, STEADY_SPEED); 
        currentDirection = 1;
    } 
    //Checken of er lijn naar links is
    else if (sensorReadings[1] >= _DEADZONEHIGH || sensorReadings[0] >= _DEADZONEHIGH) { 
        // Sterke bocht naar links
        motorControl(0, STEADY_SPEED, STEADY_SPEED, 0); 
        currentDirection = -1;
    }
    //Als er geen bochten beschikbaar is en er een lijn is rechtdoor rijden
    else if (sensorReadings[4] >= _DEADZONEHIGH && sensorReadings[3] >= _DEADZONEHIGH) { 
        // Rechtdoor rijden
        motorControl(STEADY_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 0;  
    }  
    //Lijn rechts lichte correctie naar links
    else if (sensorReadings[2] >= _DEADZONEHIGH) { 
        motorControl(SLOW_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 5;
    } 
    else if (sensorReadings[1] >= _DEADZONEHIGH) { 
        motorControl(SLOW_SPEED, 0, STEADY_SPEED, 0);  
        currentDirection = 6;
    }
    //Veel lijn links lichte correctie naar rechts
    else if (sensorReadings[5] >= _DEADZONEHIGH) { 
        motorControl(STEADY_SPEED, 0, SLOW_SPEED, 0);  
        currentDirection = 3;  
    } 
    // Te veel lijn links sterkere correctie naar rechts
    else if (sensorReadings[6] >= _DEADZONEHIGH) { 
        motorControl(STEADY_SPEED, 0, SLOWER_SPEED, 0);  
        currentDirection = 4;  
    } 
    // Alles lijnen kwijt 180 graden draaien
    else if (sum <= _DEADZONELOW * _NUM_SENSORS) { 
        // Geen lijn meer zichtbaar → dead-end
        currentDirection = 7;
        motorControl(0, SLOW_SPEED, SLOW_SPEED, 0); 
    } else {  
        // Geen sensor triggert een sterke reactie → rechtdoor blijven rijden
        motorControl(STEADY_SPEED, 0, STEADY_SPEED, 0);  
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
        case -1: blinkerLeft(); break;
        case 0: regularLight(); break;
        case 1: blinkerRight(); break;
        case 2: regularLight(); break;
        case 3: regularLight(); break;
        case 4: blinkerLeft(); break;
        case 5: regularLight(); break;
        case 6: blinkerLeft(); break;
        case 7: blinkerRight(); break;
        case 8: regularLight(); break;
        default: regularLight(); break;
    }

    //delay(50);
}

void ConeDrop() {
    unsigned long turnStartTime = 0;
    bool turning = false;

    if (_DISTANCE > 30 && _DISTANCE > 50) {
        gripperOpen();
        motorControl(STEADY_SPEED, 0, STEADY_SPEED, 0);

        if (_DISTANCE < 5) {
            gripperClosed();
            _START = true;
        }

        if (!turning) {
            turnStartTime = millis();
            turning = true;
        }
        
        if (turning && millis() - turnStartTime >= 275) {
            motorControl(0, SLOW_SPEED, SLOW_SPEED, 0); // bocht naar links
        }
        
        if (turning && millis() - turnStartTime >= 1000) {
            motorControl(STEADY_SPEED, 0, STEADY_SPEED, 0); // Vooruit rijden
            turning = false;
        }
    }

    if (_START) {
        if (sensorReadings[7] >= _DEADZONEHIGH && sensorReadings[0] >= _DEADZONEHIGH) {
            if (!_TIMERRUNNING) {
                _STARTTIME = millis(); // Start de timer
                _TIMERRUNNING = true;
            }
        } else {
            _TIMERRUNNING = false; // Reset de timer als de sensoren niets meer detecteren
            _STARTTIME = 0;
        }
        
        if (_TIMERRUNNING && millis() - _STARTTIME >= 250) {
            gripperOpen();
            motorControl(-STEADY_SPEED, 0, -STEADY_SPEED, 0); // Achteruit rijden
            _TIMERRUNNING = false; // Timer resetten na actie
        }
        
        mazeLine(); // Start de MazeLine functie als start true is
    }
}

//-----------------LICHT FUNCTIES
//-------------DEFAULT INDICATOR PROGRAM
void blinkers(int boven, int onder, bool active) {
    static unsigned long previousMillis = 0;
    static bool ledState = false;  // Ensure ledState is retained across calls
    unsigned long currentMillis = millis();

    if (active == true) {  // Alleen knipperen als "active" true is
        if (currentMillis - previousMillis >= 250) {  
            previousMillis = currentMillis;
            ledState = !ledState;  // Toggle state
        }
    } else {
        ledState = false;
        previousMillis = 0;  // LED blijft uit als knipperen stopt
    }

    if (ledState) {
        strip.setPixelColor(boven, strip.Color(255, 69, 0));  // Orange ON
        strip.setPixelColor(onder, strip.Color(255, 69, 0));  // Orange ON
    } else {
        strip.setPixelColor(boven, strip.Color(100, 0, 0));  // Red
        strip.setPixelColor(onder, strip.Color(100, 100, 100)); // White
    }

    strip.show();  // Update the strip to reflect changes
}

//-------------LEFT BLINKER
void blinkerLeft() {
    strip.clear();
    blinkers(0, 3, true);
    strip.setPixelColor(1, strip.Color(150, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.show();
}

//-------------RIGHT BLINKER
void blinkerRight() {
    strip.clear();
    blinkers(1, 2, true);
    strip.setPixelColor(0, strip.Color(150, 0, 0));  // red
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}

//-------------BRAKE LIGHT
void brakeLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(255, 0, 0));  // red
    strip.setPixelColor(1, strip.Color(255, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}

//-------------DEFAULT LIGHT
void regularLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(50, 0, 0));  // red
    strip.setPixelColor(1, strip.Color(50, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}