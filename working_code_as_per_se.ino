#include <Wire.h>
#include <IRremote.h>

// Motor pins (PWM control)
#define MOTOR_LEFT 3
#define MOTOR_RIGHT 5

// Sensor Pins
#define PIR_SENSOR A5
#define ULT_TRIG A3
#define ULT_ECHO A2
#define MMA8452Q_ADDR 0x1C  // MMA8452Q Accelerometer I2C address

// Constants
const float ACCEL_SCALE = 0.069;
const float G_TO_MPS2 = 9.80665;
const int SAFE_DISTANCE = 20;  // Safe distance in cm
const int MOTOR_MAX_SPEED = 850; // Maximum motor speed
const int MOTOR_MIN_SPEED = 238;   // Minimum speed before stopping
const int SLOWDOWN_STEP = 30;    // Speed decrement step

// Variables
float velocity = 0.0;
int currentSpeed = 0; // Tracks the motor speed

void setup() {
    Serial.begin(9600);
    pinMode(MOTOR_LEFT, OUTPUT);
    pinMode(MOTOR_RIGHT, OUTPUT);
    pinMode(PIR_SENSOR, INPUT);
    pinMode(ULT_TRIG, OUTPUT);
    pinMode(ULT_ECHO, INPUT);
    Wire.begin();
    setupMMA8452Q();
    stopMotors();
    Serial.println("System Initialized.");
}

void loop() {
    checkPIRSensor(); // Check and print PIR status
    adjustSpeedBasedOnDistance(); // Adjust speed dynamically based on ultrasonic sensor

    float accelX, accelY, accelZ;
    readAcceleration(accelX, accelY, accelZ);
    float ax_mps2 = accelX * G_TO_MPS2;

    Serial.print("Accel X: ");
    Serial.print(ax_mps2);
    Serial.print(" | Speed: ");
    Serial.println(currentSpeed);

    if (Serial.available()) {
        char command = Serial.read();
        handleCommand(command);
    }

    delay(100);
}

void handleCommand(char command) {
    switch (command) {
        case 'F':
            runMotorsForward();
            break;
        case 'B':
            runMotorsBackward();
            break;
        case 'S':
            slowDownMotors();
            break;
        default:
            Serial.println("Invalid command. Use F, B, or S.");
            break;
    }
}

void checkPIRSensor() {
    bool pirDetected = digitalRead(PIR_SENSOR);
    if (pirDetected) {
        Serial.println("Obstacle detected");
    }
}

void adjustSpeedBasedOnDistance() {
    int distance = readUltrasonicSensor();

    if (distance < SAFE_DISTANCE) {
        currentSpeed = map(distance, 0, SAFE_DISTANCE, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
        if (currentSpeed < MOTOR_MIN_SPEED) {
            currentSpeed = MOTOR_MIN_SPEED;
        }
    } else {
        currentSpeed = MOTOR_MAX_SPEED;
    }

    analogWrite(MOTOR_LEFT, currentSpeed);
    analogWrite(MOTOR_RIGHT, currentSpeed);

    Serial.print("Adjusted Speed: ");
    Serial.println(currentSpeed);
}

int readUltrasonicSensor() {
    digitalWrite(ULT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULT_TRIG, LOW);

    long duration = pulseIn(ULT_ECHO, HIGH);
    int distance = duration * 0.0343 / 2;
    Serial.print("Ultrasonic Distance: ");
    Serial.println(distance);
    return distance;
}

void runMotorsForward() {
    currentSpeed = MOTOR_MAX_SPEED;
    analogWrite(MOTOR_LEFT, currentSpeed);
    analogWrite(MOTOR_RIGHT, currentSpeed);
    Serial.println("Motors Moving Forward");
}

void runMotorsBackward() {
    currentSpeed = MOTOR_MAX_SPEED - 20; // Slightly slower for backward movement
    analogWrite(MOTOR_LEFT, currentSpeed);
    analogWrite(MOTOR_RIGHT, currentSpeed);
    Serial.println("Motors Moving Backward");
}

void slowDownMotors() {
    while (currentSpeed > MOTOR_MIN_SPEED) {
        currentSpeed -= SLOWDOWN_STEP;
        if (currentSpeed < MOTOR_MIN_SPEED) {
            currentSpeed = MOTOR_MIN_SPEED;
        }
        analogWrite(MOTOR_LEFT, currentSpeed);
        analogWrite(MOTOR_RIGHT, currentSpeed);
        Serial.print("Slowing Down... Current Speed: ");
        Serial.println(currentSpeed);
        delay(100);
    }
    Serial.println("Motors Fully Stopped");
}

void stopMotors() {
    currentSpeed = MOTOR_MIN_SPEED;
    analogWrite(MOTOR_LEFT, 0);
    analogWrite(MOTOR_RIGHT, 0);
    Serial.println("Motors Stopped");
}

void setupMMA8452Q() {
    Wire.beginTransmission(MMA8452Q_ADDR);
    Wire.write(0x2A);
    Wire.write(0x01);
    Wire.endTransmission();
}

void readAcceleration(float &ax, float &ay, float &az) {
    Wire.beginTransmission(MMA8452Q_ADDR);
    Wire.write(0x01);
    Wire.endTransmission(false);
    Wire.requestFrom(MMA8452Q_ADDR, 6);
    
    if (Wire.available() == 6) {
        int16_t rawX = (Wire.read() << 8) | Wire.read();
        int16_t rawY = (Wire.read() << 8) | Wire.read();
        int16_t rawZ = (Wire.read() << 8) | Wire.read();
        ax = rawX * ACCEL_SCALE;
        ay = rawY * ACCEL_SCALE;
        az = rawZ * ACCEL_SCALE;
    } else {
        Serial.println("Failed to read accelerometer data");
        ax = ay = az = 0;
    }
}