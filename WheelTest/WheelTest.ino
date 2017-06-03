// Pins
const int WHEEL_DIR_LB = 26;
const int WHEEL_DIR_LF = 28;
const int WHEEL_DIR_RB = 34;
const int WHEEL_DIR_RF = 36;

const int WHEEL_PWM_L = 7;
const int WHEEL_PWM_R = 8;

const int WHEEL_PWM_STBY = 33;

void leftDrive(int speed) {
  digitalWrite(WHEEL_DIR_LF, HIGH);
  digitalWrite(WHEEL_DIR_LB, LOW);
  analogWrite(WHEEL_PWM_L, speed);
}

void rightDrive(int speed) {
  digitalWrite(WHEEL_DIR_RF, HIGH);
  digitalWrite(WHEEL_DIR_RB, LOW);
  analogWrite(WHEEL_PWM_R, speed);
}

void setup() {
  // Left wheel
  pinMode(WHEEL_PWM_L, OUTPUT);
  analogWrite(WHEEL_PWM_L, 0);

  pinMode(WHEEL_DIR_LF, OUTPUT);
  pinMode(WHEEL_DIR_LB, OUTPUT);

  digitalWrite(WHEEL_DIR_LF, HIGH);
  digitalWrite(WHEEL_DIR_LB, LOW);

  // Right wheel
  pinMode(WHEEL_PWM_R, OUTPUT);
  analogWrite(WHEEL_PWM_R, 0);

  pinMode(WHEEL_DIR_RF, OUTPUT);
  pinMode(WHEEL_DIR_RB, OUTPUT);

  digitalWrite(WHEEL_DIR_RF, HIGH);
  digitalWrite(WHEEL_DIR_RB, LOW);

  // Standby
  pinMode(WHEEL_PWM_STBY, OUTPUT);
  digitalWrite(WHEEL_PWM_STBY, HIGH);

  Serial.begin(115200);
}

void loop() {
  leftDrive(100);
  rightDrive(200);
}
