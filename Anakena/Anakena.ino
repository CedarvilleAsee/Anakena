#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Servos
Servo leftScoop, rightScoop, rightDispenser, leftDispenser, dumper;
int pv;
int val_R_WALL_SENSOR;
int val_R_BARREL_SENSOR;
int val_L_BARREL_SENSOR;
int val_BACK_SENSOR;
int firstSeen = -1;
int lastSeen = -1;
int amountSeen = 0;
int p = 0;

// Drives the left wheel at a specified speed.
void leftDrive(int speed) {
  // Check the sign of the speed. If the speed is negative, we drive the
  // wheel
  // backward.
  if (speed >= 0) {
    // HIGH-LOW combination drives wheel forward.
    digitalWrite(WHEEL_DIR_LF, HIGH);
    digitalWrite(WHEEL_DIR_LB, LOW);
    analogWrite(WHEEL_SPEED_L, speed);
  }
  else {
    // LOW-HIGH combination drives wheel backward.
    digitalWrite(WHEEL_DIR_LF, LOW);
    digitalWrite(WHEEL_DIR_LB, HIGH);
    analogWrite(WHEEL_SPEED_L, -speed);    
  }
}

// Drives the right wheel at the specified speed.
void rightDrive(int speed) {
  // Check the sign of the speed. If the speed is negative, we drive the wheel
  // backward.
  if (speed >= 0) {
    // HIGH-LOW combination drives wheel forward.
    digitalWrite(WHEEL_DIR_RF, HIGH);
    digitalWrite(WHEEL_DIR_RB, LOW);
    analogWrite(WHEEL_SPEED_R, speed);
  }
  else {
    // LOW-HIGH combination drives wheel backward.
    digitalWrite(WHEEL_DIR_RF, LOW);
    digitalWrite(WHEEL_DIR_RB, HIGH);
    analogWrite(WHEEL_SPEED_R, -speed);    
  }
}

void readSensors(){

  val_R_WALL_SENSOR =   analogRead(R_WALL_SENSOR);
  val_R_BARREL_SENSOR = analogRead(R_BARREL_SENSOR);
  val_L_BARREL_SENSOR = analogRead(L_BARREL_SENSOR);
  val_BACK_SENSOR =     analogRead(BACK_SENSOR);

  // Calculate the first and last sensors which see the line.
  firstSeen = -1;
  lastSeen = -1;
  amountSeen = 0;
  for (int i = 0; i < 8; i++) {
    if (digitalRead(LINE_SENSOR[i])== 1) {
      if (firstSeen == -1) {
        firstSeen = i;
      }
      lastSeen = i;
      amountSeen++;
    }
  }
}

// Set servos and motors to their initial positions and speeds.
void resetRobot() {
  leftScoop.write(L_SCOOP_UP);
  rightScoop.write(R_SCOOP_UP);
  rightDispenser.write(R_DISPENSER_IN);
  leftDispenser.write(L_DISPENSER_IN);
  dumper.write(DUMP_DOWN);

  digitalWrite(WHEEL_DIR_LF, LOW);
  digitalWrite(WHEEL_DIR_LB, LOW);
  
  digitalWrite(WHEEL_DIR_RF, LOW);
  digitalWrite(WHEEL_DIR_RB, LOW);
  
  digitalWrite(WHEEL_STBY, HIGH);

}

// Starting state; makes sure the robot is reset.
bool start() {
  if (digitalRead(BUTTON1) == LOW) {
    return true;
  }
  else {
    resetRobot();
    pv=val_L_BARREL_SENSOR;
    return false;
  }
}

// Beginning turn from the starting position to aim toward the wall.
bool initialTurn() {
  static int startTime = -1;
  leftDrive(100);
  rightDrive(70);
  if (startTime == -1) {
    startTime = millis();
  }
  else if (millis() - startTime > 900) {
    startTime = -1;
    return true;
  }
  return false;
}

// Travel straight until the sees the wall at a reasonable distance away.
bool wallFind() {
  leftDrive(90);
  rightDrive(90);
  if (analogRead(R_WALL_SENSOR) < 780) {
    return true;
  }
  return false;
}

// Helper function for a couple of states. Wall follows at a specified distance
// from the wall.
void driveBesideWall(int targetDistance, int theSpeed) {
  int wallSensor = analogRead(R_WALL_SENSOR);
  int offset = wallSensor - targetDistance;
  int baseSpeed = theSpeed;
  int speedDiff = offset / (theSpeed/12);

  leftDrive(baseSpeed + speedDiff);
  rightDrive(baseSpeed - speedDiff);
}

// Follow the wall at a somewhat far distance for 2.5 seconds; this allows the
// robot to come parallel to the wall without danger of colliding into the
// wall.
bool farWallFollow() {

  static int startTime = -1;

  if (startTime == -1) {
    startTime = millis();
  }

  if (millis() - startTime > 1000) {
    startTime = -1;
    return true;
  }

  driveBesideWall(750,150);
  return false;
}

// Wall follow past the two first rocks. This state follows the wall closer
// than the previous state because we do not want to kick the rocks.
bool wallFollowPastRocks() {

  // Keep track of whether the sensor sees a rock.
  static bool rockSensorStatus = false;
  
  // Keep track of the times the sensor has changed whether it sees a rock.
  static int statusChangeCount = 0;

  // Get the new status from the sensor.
  int backSensor = analogRead(BACK_SENSOR);
  
  // If the status of the sensor has changed, increment the count.
  if ((backSensor < 920 && !rockSensorStatus) || 
      (backSensor > 940 && rockSensorStatus)) {
    statusChangeCount++;
    rockSensorStatus = !rockSensorStatus;
  }

  // Rock, no rock, rock, no rock -> 4 status changes -> we move to the next
  // state.
  if (statusChangeCount == 3) {
    // Reset static variables for possible next time.
    rockSensorStatus = false;
    statusChangeCount = 0;
    return true;
  }

  driveBesideWall(485,90);
  return false;
}

bool turnToFindLine() {
  leftDrive(0);
  rightDrive(100);
  if (digitalRead(LINE_SENSOR[7]) == 1) {
    return true;
  }
  return false;
}

bool clearCorner() {
  static int startTime = -1;

  leftDrive(90);
  rightDrive(90);

  if (startTime == -1) {
    startTime = millis();
  }

  if (millis() - startTime > 300) {
    startTime = -1;
    return true;
  }

  return false;
}

bool lineUpForLineFollow() {

  leftDrive(100);
  rightDrive(0);
  pv=digitalRead(LINE_SENSOR[3]);
  if (digitalRead(LINE_SENSOR[3]) == 1) {
    return true;
  }
  return false;
}

// Method that only does line following. Since we will be using line following a lot, but have different exit conditions
// we can simply call this method and leave the exit conditions to the individual states to deal with.
void rightOffsetLineFollow(){
  if (lastSeen == -1) {
    leftDrive(75);
    rightDrive(75);
  }
  else {
    leftDrive(RIGHT_OFFSET_FOLLOW_SPEEDS_L[lastSeen]);
    rightDrive(RIGHT_OFFSET_FOLLOW_SPEEDS_R[lastSeen]);
  }
}

void leftOffsetLineFollow(){
  if (lastSeen == -1) {
    leftDrive(75);
    rightDrive(75);
  }
  else {
    leftDrive(LEFT_OFFSET_FOLLOW_SPEEDS_L[lastSeen]);
    rightDrive(LEFT_OFFSET_FOLLOW_SPEEDS_R[lastSeen]);
  }
}

bool scoopRight1DeliverRight1(){
  //line follow to the barrel
  rightOffsetLineFollow();
  pv = analogRead(R_BARREL_SENSOR);
  if (analogRead(R_BARREL_SENSOR) < 500) {
    rightDispenser.write(R_DISPENSER_OUT);
    rightScoop.write(R_SCOOP_DOWN);
    return true;
  } 
  return false;
}

bool allowBarrelEjection() {
  static int startTime = -1;
  rightOffsetLineFollow();

  if (startTime == -1) {
    startTime = millis();
  }

  if (millis() - startTime > 300) {
    return true;
  }
  return false;
}

bool scoopLeft1ResetRightServos() {
  //line follow to the barrel
  leftOffsetLineFollow();
  pv = analogRead(L_BARREL_SENSOR);
  if (analogRead(L_BARREL_SENSOR) < 900) {
    leftScoop.write(L_SCOOP_DOWN);
    rightDispenser.write(R_DISPENSER_OUT);
    rightScoop.write(R_SCOOP_UP);
    return true;
  } 
  return false;
}

bool lineFollowTillNoLine(){
  rightOffsetLineFollow();
  pv = amountSeen;
  if (amountSeen == 0) {
    leftScoop.write(L_SCOOP_UP);
    leftDispenser.write(L_DISPENSER_OUT);
    rightDispenser.write(R_DISPENSER_OUT);
    return true;
  }
  return false;
}

bool wallFollowScoopRight2() {
  driveBesideWall(500, 100);
  if (val_R_BARREL_SENSOR < 700) {
    rightScoop.write(R_SCOOP_DOWN);
    return true;
  }
  return false;
}

bool wallFollowToBackWall() {
  driveBesideWall(500, 100);
  if (val_R_WALL_SENSOR < 200) {
    return true;
  }
  return false;
}

bool turnBackwardDeliverRight2() {
  leftDrive(-100);
  rightDrive(0);
  if (val_R_WALL_SENSOR > 800) {
    rightDispenser.write(R_DISPENSER_IN);
    return true;
  }
  return false;
}

bool wallFollowDeliverLeft1() {
  driveBesideWall(700, 100);
  if (val_BACK_SENSOR < 400) {
    leftDispenser.write(L_DISPENSER_IN);
    return true;
  }
  return false;
}

bool wallFollowPastIsland() {
  driveBesideWall(700, 100);
  if (val_BACK_SENSOR > 700) {
    return true;
  }
  return false;
}

bool stop() {
  leftDrive(0);
  rightDrive(0);
  return true;
}

void setup(){

  // LEDs
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);

  // Buttons
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  // Servos
  pinMode(L_SCOOP, OUTPUT);
  pinMode(R_SCOOP, OUTPUT);
  pinMode(DUMPER, OUTPUT);
  pinMode(R_DISPENSER, OUTPUT);
  pinMode(L_DISPENSER, OUTPUT);

  // Wheels
  pinMode(WHEEL_SPEED_L, OUTPUT);
  pinMode(WHEEL_DIR_LF, OUTPUT);
  pinMode(WHEEL_DIR_LB, OUTPUT);
  
  pinMode(WHEEL_SPEED_R, OUTPUT);
  pinMode(WHEEL_DIR_RF, OUTPUT);
  pinMode(WHEEL_DIR_RB, OUTPUT);
  
  pinMode(WHEEL_STBY, OUTPUT);

  // Initialize sensor pins.
  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR[i], INPUT);
  }

  pinMode(R_WALL_SENSOR, INPUT);
  pinMode(R_BARREL_SENSOR, INPUT);
  pinMode(L_BARREL_SENSOR, INPUT);
  pinMode(BACK_SENSOR, INPUT);

  // Attach servos
  leftScoop.attach(L_SCOOP);
  rightScoop.attach(R_SCOOP);
  rightDispenser.attach(R_DISPENSER);
  leftDispenser.attach(L_DISPENSER);
  dumper.attach(DUMPER);

  // Initial robot configuration
  resetRobot();
  
  // Turn  green LED  on
  digitalWrite(LEDG, HIGH);

  Serial.begin(115200);
  Serial3.begin(115200);
  Serial3.write("hello bt");
}

void breakpoint() {
  while(digitalRead(BUTTON2) == HIGH) {
    delay(50);
  }
}

void loop() {

  static int state = 0;
  readSensors();

  // Button 2 resets the state machine. This is useful as a less violent way to
  // reset the robot then using the power switch.
  if (digitalRead(BUTTON2) == LOW) state = 0;

  if(p==0){
    Serial3.print(state);
    Serial3.print(" - ");
    Serial3.println(pv);
  }
  p++;
  if(p==1000){
    p=0;
  }
    
  // State succession
  switch (state) {
    // Beginning of a run; exits when a button is pressed.
    case 0: if (start()) state++; break;

    // Drives straight; exits when wall detected.
    case 1: if (wallFind()) state++; break;

    // Follow at far distance for 1000 ms; we follow far because we don't want
    // to collide into the wall.
    case 2: if (farWallFollow()) state++; break;

    // Follow wall until three rock detection changes.
    case 3: if (wallFollowPastRocks()) state++; break;

    // Lock left wheel; exits when rightmost sensor detects line.
    case 4: if (turnToFindLine()) state++; break;

    // Drive straight for 300 ms to avoid catching corner when turning back to
    // the line.
    case 5: if (clearCorner()) state++; break;

    // Lock right wheel; exits when sensor 3 detects line.
    case 6: if (lineUpForLineFollow()) state++; break;

    // Right offset line follow; exits when right barrel detected.
    case 7: if (scoopRight1DeliverRight1()) state++; break;

    // Continues wall following for 300 ms to allow the barrel to be deposited
    // smoothly.
    case 8: if (allowBarrelEjection()) state++; break;

    // Left offset line follow; exits when left barrel detected.
    case 9:  if (scoopLeft1ResetRightServos()) state++; break;

    // Right offset line follow; resets left scoop and brings both dispensers
    // out; exits when no sensor detects the line.
    case 10: if (lineFollowTillNoLine()) state++; break;

    // Exits barrel detected.
    case 11: if (wallFollowScoopRight2()) state++; break;

    // Simple wall follow; exits on back wall. Simple wall follow; exits on when
    // the right sensor gets excessively close to the wall (should only happen
    // with the back wall).
    case 12: if (wallFollowToBackWall()) state++; break;

    // Exits when right wall sensor gets very far; delivers barrel on exit.
    case 13: if (turnBackwardDeliverRight2()) state++; break;

    // Exits and when back sensor detects island. On exit, sends left dispenser on.
    case 14: if (wallFollowDeliverLeft1()) state++; break;

    case 15: if (wallFollowPastIsland()) state++; break;
    case 16: if (stop()) state++; break;
    default:  break;
  } 
}
