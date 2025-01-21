const int SENSOR_PIN = 8; // Define the sensor pin

int lastState = HIGH;     // The previous state from the input pin
int currentState;         // The current reading from the input pin

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  // Initialize the Arduino's pin as an input
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  // Read the state of the input pin
  currentState = digitalRead(SENSOR_PIN);

  // Check for state changes and print appropriate messages
  if (lastState == HIGH && currentState == LOW) {
    Serial.println("Obstacle detected");
  } else if (lastState == LOW && currentState == HIGH) {
    Serial.println("Obstacle cleared");
  }

  // Save the last state
  lastState = currentState;

  // Add a delay to debounce the sensor reading
  delay(50);
}
