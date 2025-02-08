#include <Wire.h>

// L298N Motor Control Pins
#define ENA 9  
#define ENB 4  
#define IN1 8  
#define IN2 7  
#define IN3 6  
#define IN4 5  

// Sensor Pins
#define PIR_SENSOR A5
#define ULT_TRIG A3
#define ULT_ECHO A2
#define MMA8452Q_ADDR 0x1C  // MMA8452Q Accelerometer I2C address

// Constants
const float ACCEL_SCALE = 0.001;
const float G_TO_MPS2 = 9.80665;
const int SAFE_DISTANCE = 20;  // Safe distance in cm
const int MOTOR_MAX_SPEED = 200; // Max motor speed
const int MOTOR_MIN_SPEED = 50;   // Min speed before stopping
const int STOP_THRESHOLD = 5;     // Stop motor if distance < 5cm
const int SLOWDOWN_STEP = 10;    // Speed decrement step

// Variables
int currentSpeed = MOTOR_MAX_SPEED; // Default motor speed
bool pirDetected = false;

void setup() {
    Serial.begin(9600);

    // Motor pins setup
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Sensor setup
    pinMode(PIR_SENSOR, INPUT);
    pinMode(ULT_TRIG, OUTPUT);
    pinMode(ULT_ECHO, INPUT);
    Wire.begin();
    setupMMA8452Q();

    // Start motors moving forward
    runMotorsForward();
    Serial.println("System Initialized. Motors Running Forward.");
}

void loop() {
    pirDetected = digitalRead(PIR_SENSOR);  // Check PIR Sensor
    int distance = readUltrasonicSensor();  // Check Ultrasonic Sensor

    // **PIR SENSOR CONTROL**
    if (pirDetected) {
        stopMotors();
        Serial.println(" PIR Detected Motion! Stopping Motors.");
    } 
    // **ULTRASONIC SENSOR CONTROL**
    else {
        if (distance < STOP_THRESHOLD) {
            stopMotors();
            Serial.println("Obstacle Too Close! Stopping Motors.");
        } 
        else if (distance < SAFE_DISTANCE) {
            currentSpeed = map(distance, STOP_THRESHOLD, SAFE_DISTANCE, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
            analogWrite(ENA, currentSpeed);
            analogWrite(ENB, currentSpeed);
            Serial.print(" Slowing Down... Speed: ");
            Serial.println(currentSpeed);
        } 
        else {
            runMotorsForward(); // Move at full speed if path is clear
        }
    }

    delay(100);
}

int readUltrasonicSensor() {
    digitalWrite(ULT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULT_TRIG, LOW);

    long duration = pulseIn(ULT_ECHO, HIGH);
    int distance = duration * 0.0343 / 2;
    Serial.print(" Distance: ");
    Serial.println(distance);
    return distance;
}

void runMotorsForward() {
    currentSpeed = MOTOR_MAX_SPEED;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    Serial.println(" Motors Moving Forward");
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Motors Stopped");
}

void setupMMA8452Q() {
    Wire.beginTransmission(MMA8452Q_ADDR);
    Wire.write(0x2A);
    Wire.write(0x01);
    Wire.endTransmission();
}
