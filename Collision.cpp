int Led=13;
int Shock=2;
int x;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
}
void setup()  {
	pinMode (Led, OUTPUT);
	pinMode (Shock, INPUT);
}
void loop () {
	x=digitalRead(Shock);
	if (x==HIGH);
	digitalWrite(Led, LOW);
	digitalWrite(Led, HIGH);
}
