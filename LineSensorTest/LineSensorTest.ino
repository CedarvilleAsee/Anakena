int LINE_SENSOR[] = {39, 41, 43, 45, 47, 49, 51, 53};
int firstSeen = 9;
int lastSeen = 8;
int amountSeen = 0;

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR[i], INPUT);
  }
  
  Serial3.begin(115200);
}

void loop() {
  firstSeen = 9;
  lastSeen = 8;
  amountSeen = 0;
  for (int i = 0; i < 8; i++) {
    int reading = digitalRead(LINE_SENSOR[i]);
    if (reading == 1) {
      if (firstSeen == 9) {
        firstSeen = i;
      }
      lastSeen = i;
      amountSeen++;
    }
    Serial3.print(reading);
  }

  Serial3.print(" ");
  Serial3.print(firstSeen);
  Serial3.print(lastSeen);
  Serial3.print(amountSeen);
  Serial3.println();
  delay(200);
}
