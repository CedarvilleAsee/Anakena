int pins[] = {39, 41, 43, 45, 47, 49, 51, 53};
int numPins = 8;

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], INPUT);
  }
  
  Serial.begin(115200);
}

void loop() {
  
  for (int i = 0; i < numPins; i++) {
    Serial.print("Pin ");
    Serial.print(pins[i]);
    Serial.print(": ");
    Serial.println(digitalRead(pins[i]));
  }
  Serial.println();
  delay(300);
}
