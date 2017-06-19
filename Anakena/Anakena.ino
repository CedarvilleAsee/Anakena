#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"
//Nathan did something in this code!
// Servos
Servo leftScoop, rightScoop, rightDispenser, leftDispenser, dumper;
int printVal;
int rightWallSensorValue; 
int rightBarrelSensorValue; 
int leftBarrelSensorValue; 
int backSensorValue; 
int firstSeen = -1; 
int lastSeen = -1;
int amountSeen = 0;
int iterationCount = 0;

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

  rightWallSensorValue =   analogRead(R_WALL_SENSOR);
  rightBarrelSensorValue = analogRead(R_BARREL_SENSOR);
  leftBarrelSensorValue = analogRead(L_BARREL_SENSOR);
  backSensorValue =     analogRead(BACK_SENSOR);

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
  rightDispenser.write(R_DISPENSER_WIND_UP);
  leftDispenser.write(L_DISPENSER_KICK);
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
    printVal=rightWallSensorValue;
    return false;
  }
}

// Travel straight until the sees the wall at a reasonable distance away.
bool findNorthWall() {
  leftDrive(90);
  rightDrive(90);
  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 780) {
    return true;
  }
  return false;
}

// Helper function for a couple of states. Wall follows at a specified distance
// from the wall.
void driveBesideWall(int targetDistance, int theSpeed) {
  int wallSensor = rightWallSensorValue;
  int offset = wallSensor - targetDistance;
  int baseSpeed = theSpeed;
  int speedDiff = offset / (theSpeed/12);

  leftDrive(baseSpeed + speedDiff);
  rightDrive(baseSpeed - speedDiff);
}

// Helper function for delay based states. The first call per state is the only
// one that notices the milliseconds parameter.
bool delayState(int milliseconds) {
  static int startTime = -1;
  static int timeLimit = 0;

  if (startTime == -1) {
    startTime = millis();
    timeLimit = milliseconds;
  }

  int timeDiff = millis() - startTime;
  printVal = timeDiff;

  if (timeDiff > milliseconds) {
    startTime = -1;
    return true;
  }
  return false;
}

// Follow the wall at a somewhat far distance for 1000 milliseconds; this allows the
// robot to come parallel to the wall without danger of colliding into the
// wall.
bool farWallFollow() {
  driveBesideWall(750,150);
  return delayState(1000);
}

// Wall follow past the two first rocks. This state follows the wall closer
// than the previous state because we do not want to kick the rocks.
bool wallFollowPastRocks() {

  // Keep track of whether the sensor sees a rock.
  static bool rockSensorStatus = false;
  
  // Keep track of the times the sensor has changed whether it sees a rock.
  static int statusChangeCount = 0;
  
  // If the status of the sensor has changed, increment the count.
  if ((backSensorValue < 920 && !rockSensorStatus) || 
      (backSensorValue > 940 && rockSensorStatus)) {
    statusChangeCount++;
    rockSensorStatus = !rockSensorStatus;
  }

  printVal = statusChangeCount;
  // Rock, no rock, rock -> 3 status changes -> we move to the next
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
  printVal = lastSeen;
  if (digitalRead(LINE_SENSOR[7]) == 1) {
    return true;
  }
  return false;
}

bool clearCorner() {
  leftDrive(90);
  rightDrive(90);
  return delayState(300);
}

bool lineUpForLineFollow() {

  leftDrive(100);
  rightDrive(0);
  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 500) {
    return true;
  }
  return false;
}

void weakLineFollow(int targetLastSeen, int sensitivity, int baseSpeed = 100) {
  int offset = 0;

  if (lastSeen != -1) {
    offset = targetLastSeen - lastSeen;
  }

  int speedDiff = offset * sensitivity;

  leftDrive(baseSpeed - speedDiff);
  rightDrive(baseSpeed + speedDiff);
}

void strongLineFollow(int targetLastSeen, int sensitivity, int baseSpeed = 100) {
  static int previousLastSeen = lastSeen;
  int offset = 0;
  
  if (lastSeen != -1) {
    offset = targetLastSeen - lastSeen;
    previousLastSeen = lastSeen;
  }
  else {
    offset = targetLastSeen - previousLastSeen;
  }

  int speedDiff = offset * sensitivity;

  leftDrive(baseSpeed - speedDiff);
  rightDrive(baseSpeed + speedDiff);
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
  weakLineFollow(3, 20);


  if (rightWallSensorValue > 800) {
    rightDispenser.write(R_DISPENSER_KICK);
  }

  if (rightBarrelSensorValue < 500) {
    rightScoop.write(R_SCOOP_DOWN);
  } 
  if (delayState(1000)) {
    printVal = rightWallSensorValue;
    return true;
  }
  printVal = rightWallSensorValue;
  return false;
}

bool scoopLeft1ResetRightServos() {
  //line follow to the barrel
  weakLineFollow(6, 5);
  rightScoop.write(R_SCOOP_UP);
  printVal = leftBarrelSensorValue;
  if (leftBarrelSensorValue < 900) {
    leftScoop.write(L_SCOOP_DOWN);
  }

  if (lastSeen == -1) {
    leftDispenser.write(L_DISPENSER_WIND_UP);
    rightDispenser.write(R_DISPENSER_WIND_UP);
    return true;
  }

  return false;
}

bool wallFollowScoopRight2() {
  driveBesideWall(550, 100);
  if (rightBarrelSensorValue < 700) {
    rightScoop.write(R_SCOOP_DOWN);
  }

  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 190) {
    return true;
  }
  return false;
}

bool wallFollowToBackWall() {
  driveBesideWall(550, 100);
  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 170) {
    return true;
  }
  return false;
}

bool turnBackwardDeliverRight2() {
  leftDrive(-100);
  rightDrive(0);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 350) {
    rightDispenser.write(R_DISPENSER_KICK);
    return true;
  }
  return false;
}

bool turnAwayFromWall() {
  leftDrive(-100);
  rightDrive(0);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 800) {
    return true;
  }
  return false;
}

bool wallFollowDeliverLeft1() {
  driveBesideWall(700, 100);
  printVal = backSensorValue;
  if (backSensorValue < 400) {
    leftDispenser.write(L_DISPENSER_KICK);
    rightScoop.write(R_SCOOP_UP);
    rightDispenser.write(R_DISPENSER_WIND_UP);
    return true;
  }
  return false;
}

bool wallFollowToCorner() {
  driveBesideWall(700, 100);
  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 230) {
    return true;
  }
  return false;
}

bool lineUpForRightScoop3() {
  leftDrive(-100);
  rightDrive(0);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 400) {
    return true;
  }
  return false;
}

bool passCornerDeliverRight3() {
  leftDrive(100);
  rightDrive(100);

  if (rightBarrelSensorValue < 700) {
    rightScoop.write(R_SCOOP_DOWN);
    rightDispenser.write(R_DISPENSER_KICK);
  }

  if (delayState(350)) {
    rightDispenser.write(R_DISPENSER_KICK);
    return true;
  }
  return false;
}

bool passCornerScoopRight3() {
  leftDrive(100);
  rightDrive(100);
  
  if (rightBarrelSensorValue < 700) {
    rightScoop.write(R_SCOOP_DOWN);
    rightDispenser.write(R_DISPENSER_KICK);
  }
  return delayState(150);
}

bool skimSouthWall() {
  leftDrive(100);
  rightDrive(100);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 700) {
    return true;
  }
  return false;
}

bool findSouthLine() {
  leftDrive(0);
  rightDrive(100);

  printVal = lastSeen;
  if (lastSeen != -1) {
    leftScoop.write(L_SCOOP_UP);
    leftDispenser.write(L_DISPENSER_WIND_UP);
    return true;
  }
  return false;
}

bool alignWithSouthLine() {
  strongLineFollow(3, 20);
  delayState(300);
}

bool scoopLeft2DeliverLeft2() {
  strongLineFollow(7, 20);
  
  printVal = backSensorValue;
  if (backSensorValue < 700) {
    leftScoop.write(L_SCOOP_DOWN);
    leftDispenser.write(L_DISPENSER_KICK);
    rightScoop.write(R_SCOOP_UP);
    rightDispenser.write(R_DISPENSER_WIND_UP);

    return true;
  }

  return false;
}

bool allowLeftBarrelEjection2() {
  // Make line following go slower.
  weakLineFollow(6, 10, 50);
  return delayState(600);
}

bool lineFollowToIsland() {
  rightOffsetLineFollow();
  return rightWallSensorValue < 500;
}

bool scoopRight4() {
  driveBesideWall(500, 100);

  printVal = rightBarrelSensorValue;
  if (rightBarrelSensorValue < 500) {
    rightScoop.write(R_SCOOP_DOWN);
  }

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 800) {
    leftScoop.write(L_SCOOP_UP);
    return true;
  }

  return false;
}

bool allowRightBarrelEjection4() {
  rightOffsetLineFollow();

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 800) {
    return true;
  }
  return false;
  
}

bool wigglePart1() {
  leftDrive(0);
  rightDrive(60);
  printVal = lastSeen;
  return lastSeen == -1;
}

bool wigglePart2DeliverRight4() {
  leftDrive(60);
  rightDrive(0);
  printVal = lastSeen;
  if (lastSeen == 2) {
    rightDispenser.write(R_DISPENSER_KICK);
  }

  return lastSeen == 0;
}

bool wigglePart3() {
  leftDrive(0);
  rightDrive(60);
  rightDispenser.write(R_DISPENSER_KICK);
  printVal = lastSeen;
  return lastSeen == 4;
}

bool hugCorner() {
  leftDrive(60);
  rightDrive(0);
  printVal = rightWallSensorValue;
  return rightWallSensorValue < 570;
}

bool findSouthWall() {
  leftDrive(80);
  rightDrive(80);
  printVal = rightWallSensorValue;
  return rightWallSensorValue < 220;
}

bool smearWall() {
  driveBesideWall(250, 50);
  printVal = rightWallSensorValue;
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
  static int lastState = 0;
  readSensors();

  // Button 2 resets the state machine. This is useful as a less violent way to
  // reset the robot then using the power switch.
  if (digitalRead(BUTTON2) == LOW) state = 0;
    
  // State succession
  switch (state) {
    // Beginning of a run; exits when a button is pressed.
    case 0: if (start()) state++; break;

    // Drives straight; exits when wall detected.
    case 1: if (findNorthWall()) state++; break;

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

    // Left offset line follow; exits when left barrel detected.
    case 8:  if (scoopLeft1ResetRightServos()) state++; break;

    // Exits when barrel detected.
    case 9: if (wallFollowScoopRight2()) state++; break;

    // Exits when right wall sensor gets a short distance away; delivers barrel
    // on exit.
    case 10: if (turnBackwardDeliverRight2()) state++; break;

    // Exits when the right wall sensor gets a far distance away; does nothing
    // significant.
    case 11: if (turnAwayFromWall()) state++; break;

    // Exits and when back sensor detects island. On exit, sends left dispenser on.
    case 12: if (wallFollowDeliverLeft1()) state++; break;

    case 13: if (wallFollowToCorner()) state++; break;
    case 14: if (lineUpForRightScoop3()) state++; break;
    case 15: if (passCornerDeliverRight3()) state++; break;
    case 16: if (passCornerScoopRight3()) state++; break;
    case 17: if (skimSouthWall()) state++; break;
    case 18: if (findSouthLine()) state++; break;
    case 19: if (alignWithSouthLine()) state++; break;
    case 20: if (scoopLeft2DeliverLeft2()) state++; break;
    case 21: if (allowLeftBarrelEjection2()) state++; break;
    case 22: if (lineFollowToIsland()) state++; break;
    case 23: if (scoopRight4()) state++; break;
    case 24: if (wigglePart1()) state++; break;
    case 25: if (wigglePart2DeliverRight4()) state++; break;
    case 26: if (wigglePart3()) state++; break;
    case 27: if (hugCorner()) state++; break;
    case 28: if (findSouthWall()) state++; break;
    case 29: if (smearWall()) state++; break;
    case 30: if (stop()) state++; break;
    default: break;
  } 

  if(iterationCount % 1000 == 0 || lastState != state){
    Serial3.print(state);
    Serial3.print(" - ");
    Serial3.println(printVal);
    lastState = state;
  }
  iterationCount++;
}
