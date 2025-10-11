int digitalPin = 7;
int threshold = 450;

void setup() {
    Serial.begin(9600);
    pinMode(digitalPin, OUTPUT);
}

void loop() {
    int value = analogRead(A0);

    Serial.print("light Value: ");
    Serial.println(value);
  if(value > threshold){
  	digitalWrite(digitalPin, LOW);
  }else {
  	digitalWrite(digitalPin, HIGH);
  }
}
