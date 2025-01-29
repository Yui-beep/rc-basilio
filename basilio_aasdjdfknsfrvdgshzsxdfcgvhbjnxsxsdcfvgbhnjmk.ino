#include <Wire.h>
#include <IRremote.h>

// Motor pins (using PWM pins for speed control)
#define MOTOR_LEFT 3   // PWM pin
#define MOTOR_RIGHT 5  // PWM pin

// Sensor Pins
#define PIR_SENSOR A5
#define ULT_TRIG A3
#define ULT_ECHO A2
#define MMA8452Q_ADDR 0x1C  // MMA8452Q Accelerometer I2C address

// Constants for MMA8452Q
const float ACCEL_SCALE = 0.001;  // Convert raw readings to g's
const float G_TO_MPS2 = 9.80665;  // Convert g to m/s²

// Variables for velocity calculation
float velocity = 0.0;
long lastTime = 0;

// IR remote setup (not active in this final setup)
#define IR_RECEIVER_PIN 11
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Set motor pins as outputs (PWM control)
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);

  // Sensor pins as inputs
  pinMode(PIR_SENSOR, INPUT);
  pinMode(ULT_TRIG, OUTPUT);
  pinMode(ULT_ECHO, INPUT);

  // Initialize the MMA8452Q accelerometer
  Wire.begin();
  setupMMA8452Q();
  
  // Initial motor state
  stopMotors();

  // Initial message
  Serial.println("System Initialized.");
}

void loop() {
  // Consolidated sensor readings
  bool obstacleDetected = readAllSensors();

  // Read Accelerometer data (X, Y, Z)
  float ax, ay, az;
  readAcceleration(ax, ay, az);  // Read accelerometer data
  
  // Calculate acceleration in m/s²
  float ax_mps2 = ax * G_TO_MPS2;
  
  // Print accelerometer data (X, Y, Z, Speed)
  Serial.print("Accel X: ");
  Serial.print(ax_mps2);
  Serial.print(" | Speed: ");
  Serial.println(velocity);

  // Stop motors if an obstacle is detected
  if (obstacleDetected) {
    stopMotors();
  }

  // Movement based on sensor readings (example: no obstacle detected)
  if (!obstacleDetected) {
    runMotorsForward();
  }

  // Delay for sensor stability and smooth operation
  delay(100);
}

bool readAllSensors() {
  bool pirDetected = digitalRead(PIR_SENSOR);
  bool ultrasonicDetected = readUltrasonicSensor();

  // Print sensor states
  Serial.print("PIR: ");
  Serial.print(pirDetected);
  Serial.print(" | US: ");
  Serial.println(ultrasonicDetected);

  return pirDetected || ultrasonicDetected;
}

bool readUltrasonicSensor() {
  digitalWrite(ULT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULT_TRIG, LOW);

  long duration = pulseIn(ULT_ECHO, HIGH);
  int distance = duration * 0.0343 / 2;  // Convert to cm

  // Print ultrasonic distance
  Serial.print("Ultrasonic Distance: ");
  Serial.println(distance);

  return (distance > 0 && distance <= 20);  // Set threshold for obstacle detection
}

void runMotorsForward() {
  int motorSpeed = 150;  // Set motor speed (0 to 255)
  analogWrite(MOTOR_LEFT, motorSpeed);  // Set motor left speed
  analogWrite(MOTOR_RIGHT, motorSpeed); // Set motor right speed
  Serial.println("Motors Moving Forward");
}

void stopMotors() {
  analogWrite(MOTOR_LEFT, 0);   // Stop motor left
  analogWrite(MOTOR_RIGHT, 0);  // Stop motor right
  Serial.println("Motors Stopped");
}

void setupMMA8452Q() {
  Wire.beginTransmission(MMA8452Q_ADDR);
  Wire.write(0x2A);  // CTRL_REG1
  Wire.write(0x01);  // Set active mode
  Wire.endTransmission();
}

void readAcceleration(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MMA8452Q_ADDR);
  Wire.write(0x01);  // Start reading from OUT_X_MSB
  Wire.endTransmission(false);
  Wire.requestFrom(MMA8452Q_ADDR, 6);

  if (Wire.available() == 6) {
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();

    ax = rawX * ACCEL_SCALE;
    ay = rawY * ACCEL_SCALE;
    az = rawZ * ACCEL_SCALE;
  }
}
