const int R_WALL_SENSOR = A0;
const int R_ROCK_SENSOR = A1;
const int L_ROCK_SENSOR = A6;
const int BACK_SENSOR = A7;

void setup() {
  Serial3.begin(115200);
  pinMode(R_WALL_SENSOR, INPUT);
  pinMode(R_ROCK_SENSOR, INPUT);
  pinMode(L_ROCK_SENSOR, INPUT);
  pinMode(BACK_SENSOR, INPUT);
}

void loop() {
  Serial3.print("Right wall: ");
  Serial3.println(analogRead(R_WALL_SENSOR));

  Serial3.print("Right rock: ");
  Serial3.println(analogRead(R_ROCK_SENSOR));

  Serial3.print("Left rock: ");
  Serial3.println(analogRead(L_ROCK_SENSOR));

  Serial3.print("Back senser: ");
  Serial3.println(analogRead(BACK_SENSOR));
  
  Serial3.println();
  delay(300);
}
