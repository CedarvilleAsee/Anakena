const int R_WALL_SENSOR = A0;
const int R_ROCK_SENSOR = A1;
const int L_ROCK_SENSOR = A6;
const int BACK_SENSOR = A7;

void setup() {
  Serial.begin(115200);
  pinMode(R_WALL_SENSOR, INPUT);
  pinMode(R_ROCK_SENSOR, INPUT);
  pinMode(L_ROCK_SENSOR, INPUT);
  pinMode(BACK_SENSOR, INPUT);
}

void loop() {
  Serial.print("Right wall: ");
  Serial.println(analogRead(R_WALL_SENSOR));

  Serial.print("Right rock: ");
  Serial.println(analogRead(R_ROCK_SENSOR));

  Serial.print("Left rock: ");
  Serial.println(analogRead(L_ROCK_SENSOR));

  Serial.print("Back senser: ");
  Serial.println(analogRead(BACK_SENSOR));
  
  Serial.println();
  delay(300);
}
