int counter = 0;
char tempString[10];
void setup() {
  Serial3.begin(9600);
  Serial3.write(0x76);
}

void loop() {
  counter++;
  sprintf(tempString, "%4d", counter);
  Serial3.print(tempString);
  delay(500);
}
