const int SENSOR_PIN = 8; // Define the sensor pin correctly

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  // Initialize the Arduino's pin as an input
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  static int lastState = HIGH; // Store the last state of the sensor
  int currentState = digitalRead(SENSOR_PIN); // Read the current state

  // Only print when the state changes
  if (currentState != lastState) {
    if (currentState == LOW) {
      Serial.println("The obstacle is present");
    } else {
      Serial.println("The obstacle is NOT present");
    }
    lastState = currentState; // Update the last state
  }

  // Optional: Reduce delay if faster response is needed
  delay(50); // Adjust delay as necessary
}
