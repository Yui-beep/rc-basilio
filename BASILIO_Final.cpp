/*  BASILIO
/*  Created on: 2024-2025   
 *
 */

#include <SimpleDHT.h> 

// Collision Sensor
#define COL_SENSOR

// Motion sensor (PIR)
#define PIR_SENSOR      A5 // PIR sensor (pins:1)

// Ultrasonic sensor (pins:2)
#define ULT_TRIG        A3
#define ULT_ECHO        A2
#define IR_SENSOR       A1 // IR sensor (pins:1)

// leds (pins:5)
#define IR_LED          9
#define ULT_LED         10
#define PIR_LED         13

void setup() {
  // Begin serial communication for Serial Monitor
  Serial.begin(9600);
  
  // set sensor pins as inputs
  pinMode(PIR_SENSOR, INPUT);
    
  // set trigger pin as output for ultrasonic sensor
  pinMode(ULT_TRIG, OUTPUT);
  
  // set led pins as output
  pinMode(IR_LED, OUTPUT);
  pinMode(ULT_LED, OUTPUT);
  pinMode(PIR_LED, OUTPUT);

  // Initial message on Serial Monitor
  Serial.println("Sensor Readings:");
  Serial.println("IR= (IR Sensor Data)");
  Serial.println("US= (Ultrasonic Sensor Data)");
  Serial.println("PS= (PIR Sensor Data)");
}

void loop() {
  /*
     read continuously from each sensor
  */
  // print digital values from IR Sensor
  ir_Readings();
  // distance measurement using Ultrasonic Sensor
  us_Readings();
  // read values from PIR sensor
  pir_Readings();
  // Removed HT sensor readings as HT_SENSOR is no longer used
}

void ir_Readings() {
  int x = digitalRead(IR_SENSOR);
  // Print the IR sensor data to the Serial Monitor
  Serial.print("IR=");
  Serial.println(!x);
  
  // IR Sensor input condition for LED to glow
  if (x == 0) {
    digitalWrite(IR_LED, HIGH);
  } else {
    digitalWrite(IR_LED, LOW);
  }
}

void us_Readings() {
  long duration;
  int distance;
  duration = time_Measurement(duration);
  distance = (int)duration * (0.0343) / 2;

  if (distance > 99 || distance < 0) { // If distance is negative or greater than 99, then always show distance = 0
    distance = 0;
  }

  display_distance(distance);
}

void pir_Readings() {
  int x = digitalRead(PIR_SENSOR);
  // Print the PIR sensor data to the Serial Monitor
  Serial.print("PS=");
  Serial.println(x);
  
  // PIR Sensor input condition for LED to glow
  if (x == 1) {
    digitalWrite(PIR_LED, HIGH);
  } else {
    digitalWrite(PIR_LED, LOW);
  }
}

// Removed HT sensor readings as HT_SENSOR is no longer used

int time_Measurement(int duration) {
  digitalWrite(ULT_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(ULT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULT_TRIG, LOW);

  duration = pulseIn(ULT_ECHO, HIGH);
  return duration;
}

void display_distance(int distance) {
  // Print the ultrasonic sensor distance to the Serial Monitor
  Serial.print("US=");
  Serial.println(distance);

  // Distance condition for LED to glow
  if (1 <= distance && distance <= 10) {
    digitalWrite(ULT_LED, HIGH);
    tone(BUZZER, 2000);
  } else {
    digitalWrite(ULT_LED, LOW);
    noTone(BUZZER);
  }

  delay(10);
}
