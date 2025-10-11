const int ledPin = 13;
const int signalPin = 8;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(signalPin, INPUT);
}

void loop() {
  int roverSignal = digitalRead(signalPin);

  if (roverSignal == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  
  delay(50);
}