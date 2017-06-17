// Wheels
const int WHEEL_DIR_LB = 26;
const int WHEEL_DIR_LF = 28;
const int WHEEL_DIR_RB = 34;
const int WHEEL_DIR_RF = 36;

const int WHEEL_SPEED_L = 7;
const int WHEEL_SPEED_R = 8;

const int WHEEL_STBY = 33;


void leftDrive(int speed) {
  // HIGH-LOW combination drives wheel forward.
  digitalWrite(WHEEL_DIR_LF, HIGH);
  digitalWrite(WHEEL_DIR_LB, LOW);
  analogWrite(WHEEL_SPEED_L, speed);
}

void rightDrive(int speed) {
  // HIGH-LOW combination drives wheel forward.
  digitalWrite(WHEEL_DIR_RF, HIGH);
  digitalWrite(WHEEL_DIR_RB, LOW);
  analogWrite(WHEEL_SPEED_R, speed);
}

void setup() {
  // Wheels
  pinMode(WHEEL_SPEED_L, OUTPUT);
  pinMode(WHEEL_DIR_LF, OUTPUT);
  pinMode(WHEEL_DIR_LB, OUTPUT);
  
  pinMode(WHEEL_SPEED_R, OUTPUT);
  pinMode(WHEEL_DIR_RF, OUTPUT);
  pinMode(WHEEL_DIR_RB, OUTPUT);
  
  pinMode(WHEEL_STBY, OUTPUT);
  
  digitalWrite(WHEEL_DIR_LF, LOW);
  digitalWrite(WHEEL_DIR_LB, LOW);
  
  digitalWrite(WHEEL_DIR_RF, LOW);
  digitalWrite(WHEEL_DIR_RB, LOW);
  
  digitalWrite(WHEEL_STBY, HIGH);
  Serial.begin(115200);
}

void loop() {
  Serial.println("hello");
  leftDrive(100);
  rightDrive(100);
}
