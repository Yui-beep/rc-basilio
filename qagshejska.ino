#include <Wire.h>  // Library for I2C communication (Accelerometer)

// L298N Motor Driver Pinouts for Arduino Uno
#define ENA 9   // PWM for Motor A (Arduino Uno supports PWM on pin 9)
#define ENB 10  // PWM for Motor B (Arduino Uno supports PWM on pin 10)
#define IN1 8   // Motor A Direction
#define IN2 7   // Motor A Direction
#define IN3 6   // Motor B Direction
#define IN4 5   // Motor B Direction

// PIR Motion Sensor
#define PIR_SENSOR 2  // Digital input from PIR

// Ultrasonic Sensor (HC-SR04)
#define ULT_TRIG 12   // Trigger pin
#define ULT_ECHO 11   // Echo pin

// Accelerometer (MMA8452Q) - I2C Pins (Arduino Uno uses A4 for SDA, A5 for SCL)
#define MMA8452Q_ADDR 0x1D  // Default I2C address (could be 0x1C if configured)

// Constants & Variables
const int MOTOR_MAX_SPEED = 255;  // Full speed (PWM range 0-255)
const int MOTOR_STOP = 0;         // Motor off
const int SAFE_DISTANCE = 20;     // cm (if an obstacle is within this range, slow down)
const int STOP_THRESHOLD = 5;     // cm (if an obstacle is this close, stop immediately)
const float TILT_THRESHOLD = 0.5; // g (if tilt exceeds ±0.5g, stop motors)

bool pirDetected = false;         // PIR motion detection flag
int currentSpeed = MOTOR_MAX_SPEED;  // Stores current speed

void setup() {
  Serial.begin(9600);

  // Set up motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set up sensor pins
  pinMode(PIR_SENSOR, INPUT);
  pinMode(ULT_TRIG, OUTPUT);
  pinMode(ULT_ECHO, INPUT);

  // Initialize I2C for Accelerometer
  Wire.begin();
  setupMMA8452Q();

  // Start Motors at Full Speed
  runMotorsForward(MOTOR_MAX_SPEED);
  Serial.println("System Initialized. Motors Running at Full Speed.");
}

void loop() {
  pirDetected = stablePIRDetection();  // Read PIR Sensor with debounce
  int distance = readUltrasonicSensor(); // Read Ultrasonic Sensor
  float tilt = readAccelerometerTilt();  // Read Accelerometer Tilt (X-axis)

  // Debug output
  Serial.print("PIR: ");
  Serial.print(pirDetected ? "Motion Detected" : "No Motion");
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Tilt: ");
  Serial.print(tilt, 3);
  Serial.print(" g | Motor Speed: ");
  Serial.println(currentSpeed);

  // Control logic:
  if (pirDetected) {
    stopMotors();
    Serial.println("PIR Detected Motion. Stopping Motors.");
  }
  else if (distance < STOP_THRESHOLD) {
    stopMotors();
    Serial.println("Obstacle Too Close. Stopping Motors.");
  }
  else if (distance < SAFE_DISTANCE) {
    int targetSpeed = map(distance, STOP_THRESHOLD, SAFE_DISTANCE, 100, MOTOR_MAX_SPEED);
    changeSpeedGradually(targetSpeed);
    Serial.println("Slowing Down...");
  }
  else if (tilt > TILT_THRESHOLD || tilt < -TILT_THRESHOLD) {
    stopMotors();
    Serial.println("Tilt Detected. Stopping Motors for Safety.");
  }
  else {
    changeSpeedGradually(MOTOR_MAX_SPEED);
  }

  delay(200);  // Delay for stable data logging
}

// Read the ultrasonic sensor and return the distance in cm
int readUltrasonicSensor() {
  static int lastValidDistance = SAFE_DISTANCE; // Fallback distance

  digitalWrite(ULT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULT_TRIG, LOW);

  long duration = pulseIn(ULT_ECHO, HIGH, 30000);  // 30ms timeout
  if (duration == 0) {
    return lastValidDistance;
  }
  int distance = duration * 0.0343 / 2; // Convert to cm
  lastValidDistance = distance;
  return distance;
}

// Debounce function for PIR sensor to reduce false triggers
bool stablePIRDetection() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(PIR_SENSOR)) count++;
    delay(20);
  }
  return (count >= 3);
}

// Set motor direction and apply speed via PWM
void runMotorsForward(int speed) {
  currentSpeed = speed;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Stop both motors
void stopMotors() {
  currentSpeed = MOTOR_STOP;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_STOP);
  analogWrite(ENB, MOTOR_STOP);
}

// Gradually adjust the motor speed for smooth acceleration/deceleration
void changeSpeedGradually(int targetSpeed) {
  while (currentSpeed != targetSpeed) {
    if (currentSpeed < targetSpeed) {
      currentSpeed += 5;
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else {
      currentSpeed -= 5;
      if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }
    runMotorsForward(currentSpeed);
    delay(50); // Delay to smooth the transition
  }
}

// Initialize the MMA8452Q accelerometer
void setupMMA8452Q() {
  Wire.beginTransmission(MMA8452Q_ADDR);
  Wire.write(0x2A);  // Control register address
  Wire.write(0x01);  // Set Active Mode
  Wire.endTransmission();
}

// Read accelerometer tilt on the X-axis (using 2 bytes, 12-bit value)
float readAccelerometerTilt() {
  Wire.beginTransmission(MMA8452Q_ADDR);
  Wire.write(0x01);  // Starting register for X-axis MSB
  Wire.endTransmission(false);
  Wire.requestFrom(MMA8452Q_ADDR, 2);  // Request 2 bytes

  if (Wire.available() >= 2) {
    int16_t rawData = (Wire.read() << 8) | Wire.read();
    return rawData * 0.000244;  // Convert raw data to g (assuming 2g sensitivity)
  }
  return 0.0;
}