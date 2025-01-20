#define ECHO_PIN 4
#define TRIG_PIN 5

long duration; 
int distance;

void setup() {
    pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an OUTPUT
    pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an INPUT

    // Start Serial Communication at 9600 baud rate
    Serial.begin(9600);
    Serial.println("Distance measurement using Arduino Uno.");
    delay(500);
}

void loop() {
    // Clear the trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2); // wait for 2 microseconds

    // Trigger the sensor
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); // keep the trigger "ON" for 10 microseconds
    digitalWrite(TRIG_PIN, LOW); // Turn off the pulse trigger

    // Measure the duration of the echo
    duration = pulseIn(ECHO_PIN, HIGH);

    // Check for valid duration
    if (duration == 0) {
        Serial.println("Error: No echo received.");
    } else {
        // Calculate distance
        distance = duration * 0.0344 / 2; // Convert duration to distance
        Serial.print("Distance: ");
        Serial.print(distance); // Print the output in serial monitor
        Serial.println(" cm");
    }

    delay(100); // Wait before the next measurement
}
