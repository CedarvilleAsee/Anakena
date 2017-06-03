int pins[] = {37, 39, 41, 43, 45, 47, 49, 51, 50, 48, 46, 44, 42};
int numPins = 13;

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], INPUT);
  }
  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  for (int i = 0; i < numPins; i++) {
    Serial.print("Pin ");
    Serial.print(pins[i]);
    Serial.print(": ");
    Serial.println(digitalRead(pins[i]));
  }
  Serial.println();
  delay(300);
}
