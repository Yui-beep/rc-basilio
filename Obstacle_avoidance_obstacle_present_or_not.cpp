const int SENSOR_PIN = 8;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize the Arduino's pin as aninput
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  int state = digitalRead(SENSOR_PIN);
  if (state == LOW)
    Serial.println("The obstacle is present");
  else
    Serial.println("The obstacle is NOT present");
  delay(100);
}
