#include <IRremote.h>

// IR Receiver pin
#define IR_RECEIVER_PIN 11

// Motor pins
#define MOTOR_LEFT 5
#define MOTOR_RIGHT 6

// Sensor Pins
#define CS_SENSOR 2
#define PIR_SENSOR A5
#define ULT_TRIG A3
#define ULT_ECHO A2
#define IR_SENSOR A1

// Define IR codes (replace with your phone's IR codes after decoding)
#define IR_FORWARD 0xFFA25D  // Example code for "Forward"
#define IR_BACKWARD 0xFFE21D // Example code for "Backward"

IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Start the IR receiver
  irrecv.enableIRIn();

  // Set motor pins as outputs
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);

  // Sensor pins as inputs
  pinMode(CS_SENSOR, INPUT);
  pinMode(PIR_SENSOR, INPUT);
  pinMode(IR_SENSOR, INPUT);
  pinMode(ULT_TRIG, OUTPUT);
  pinMode(ULT_ECHO, INPUT);

  // Initial motor state
  stopMotors();

  // Initial message
  Serial.println("System Initialized.");
}

void loop() {
  // Consolidated sensor readings
  bool obstacleDetected = readAllSensors();

  if (irrecv.decode(&results)) {
    // Print the received IR code
    Serial.print("IR Code: ");
    Serial.println(results.value, HEX);

    // Act based on the received IR code if no obstacle is detected
    if (!obstacleDetected) {
      if (results.value == IR_FORWARD) {
        runMotorsForward();
      } else if (results.value == IR_BACKWARD) {
        runMotorsBackward();
      }
    } else {
      stopMotors();
    }

    // Resume the IR receiver
    irrecv.resume();
  }

  // Stop motors if an obstacle is detected
  if (obstacleDetected) {
    stopMotors();
  }
}

bool readAllSensors() {
  bool collisionDetected = digitalRead(CS_SENSOR);
  bool pirDetected = digitalRead(PIR_SENSOR);
  bool irDetected = digitalRead(IR_SENSOR) == LOW;
  bool ultrasonicDetected = readUltrasonicSensor();

  // Print all sensor states
  Serial.print("CS: ");
  Serial.print(collisionDetected);
  Serial.print(" | PIR: ");
  Serial.print(pirDetected);
  Serial.print(" | IR: ");
  Serial.print(irDetected);
  Serial.print(" | US: ");
  Serial.println(ultrasonicDetected);

  return collisionDetected || pirDetected || irDetected || ultrasonicDetected;
}

bool readUltrasonicSensor() {
  digitalWrite(ULT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULT_TRIG, LOW);

  long duration = pulseIn(ULT_ECHO, HIGH);
  int distance = duration * 0.0343 / 2;

  // Print ultrasonic distance
  Serial.print("Ultrasonic Distance: ");
  Serial.println(distance);

  return (distance > 0 && distance <= 10);
}

void runMotorsForward() {
  digitalWrite(MOTOR_LEFT, HIGH);
  digitalWrite(MOTOR_RIGHT, HIGH);
  Serial.println("Moving Forward");
}

void runMotorsBackward() {
  digitalWrite(MOTOR_LEFT, LOW);
  digitalWrite(MOTOR_RIGHT, HIGH); // Reverse polarity example
  Serial.println("Moving Backward");
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT, LOW);
  digitalWrite(MOTOR_RIGHT, LOW);
  Serial.println("Motors Stopped");
}
