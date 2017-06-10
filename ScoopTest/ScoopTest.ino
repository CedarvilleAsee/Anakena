#include <Servo.h>

Servo scooper;

void setup() {
  pinMode(30, INPUT_PULLUP);
  pinMode(12, OUTPUT);
  scooper.attach(12);
}

void loop() {
  if (digitalRead(30) == LOW) {
    scooper.write(112);
    delay(1000);
    scooper.write(80);
  }
}
