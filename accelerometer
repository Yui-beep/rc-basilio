int xReading = 0;
int yReading = 0;
int zReading = 0;
void setup()
{
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
Serial.begin(9600);
pinMode(2, OUTPUT);
pinMode(3, OUTPUT);
pinMode(4, OUTPUT);
}
void loop()
{
xReading = analogRead(A0);
yReading = analogRead(A1);
zReading = analogRead(A2);
Serial.print("X :");
Serial.print(xReading);
Serial.print("\tY: ");
Serial.print(yReading);
Serial.print("\tZ: ");
Serial.println(zReading);
if (xReading >= 800) {
digitalWrite(2, HIGH);
} else {
digitalWrite(2, LOW);
}
if (yReading >= 800) {
digitalWrite(3, HIGH);
} else {
digitalWrite(3, LOW);
}
if (zReading >= 800) {
digitalWrite(4, HIGH);
} else {
digitalWrite(4, LOW);
}
delay(1000); // Wait for 1000 millisecond(s)
}
