#include <Wire.h>
#include <IRremote.h>

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
const int SLOWDOWN_STEP = 10;    // Speed decrement step

// Variables
float velocity = 0.0;
int currentSpeed = MOTOR_MAX_SPEED; // Default motor speed

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
    checkPIRSensor(); // Check PIR and print if obstacle is detected
    adjustSpeedBasedOnDistance(); // Adjust speed dynamically based on ultrasonic

    float accelX, accelY, accelZ;
    readAcceleration(accelX, accelY, accelZ);
    float ax_mps2 = accelX * G_TO_MPS2;

    Serial.print("Accel X: ");
    Serial.print(ax_mps2);
    Serial.print(" | Speed: ");
    Serial.println(currentSpeed);

    delay(100);
}

void checkPIRSensor() {
    bool pirDetected = digitalRead(PIR_SENSOR);
    if (pirDetected) {
        Serial.println("Obstacle detected (PIR Sensor)");
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

    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);

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
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    Serial.println("Motors Moving Forward");
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
