 #define echoPin  
   4
#define trigPin                                            \
    5                                \


long duration; 
int distance;
void setup()
{
    pinMode(trigPin,
            OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

    // Serial Communication is starting with 9600 of
    // baudrate speed
    Serial.begin(9600);

    // The text to be printed in serial monitor
    Serial.println(
        "Distance measurement using Arduino Uno.");
    delay(500);
}

void loop()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // wait for 2 ms to avoid
                          // collision in serial monitor

    digitalWrite(
        trigPin,
        HIGH); // turn on the Trigger to generate pulse
    delayMicroseconds(
        10); // keep the trigger "ON" for 10 ms to generate
            
    digitalWrite(trigPin,
                 LOW); // Turn off the pulse trigger to stop
                    


    duration = pulseIn(echoPin, HIGH);
    distance
        = duration * 0.0344 / 2; // Expression to calculate
                                
    Serial.print("Distance: ");
    Serial.print(
        distance); // Print the output in serial monitor
    Serial.println(" cm");
    delay(100);
}
