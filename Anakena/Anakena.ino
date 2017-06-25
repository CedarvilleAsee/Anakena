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
int longSensorValue;
int backSensorValue; 
int firstSeen = -1; 
int lastSeen = -1;
int amountSeen = 0;
int iterationCount = 0;

// Drives the left wheel at a specified speed.
void leftDrive(int speed) {
  if (speed > 254) speed = 254;
  if (speed < -254) speed = -254;

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
  if (speed > 254) speed = 254;
  if (speed < -254) speed = -254;

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
  longSensorValue = analogRead(LONG_SENSOR);

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
  leftScoop.write(L_SCOOP_DOWN);
  rightScoop.write(R_SCOOP_DOWN);
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
    printVal = longSensorValue;
    return false;
  }
}

// Travel straight until the sees the wall at a reasonable distance away.
bool findNorthWall() {
  leftDrive(140);
  rightDrive(140);
  leftScoop.write(L_SCOOP_UP);
  rightScoop.write(R_SCOOP_UP);
  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 780) {
    return true;
  }
  return false;
}

// Helper function for a couple of states. Wall follows at a specified distance
// from the wall.
void driveBesideWall(int targetDistance, int baseSpeed, int sensitivity) {
  int wallSensor = rightWallSensorValue;
  int offset = wallSensor - targetDistance;
  int speedDiff = offset * sensitivity / 20;

  leftDrive(baseSpeed + speedDiff);
  rightDrive(baseSpeed - speedDiff);
}

void weakLineFollow(int targetLastSeen, int sensitivity, int baseSpeed = 100) {
  int offset = 0;
  if (targetLastSeen == 7 && lastSeen == -1) {
    offset = -2;
  }
  else if (lastSeen != -1) {
    offset = targetLastSeen - lastSeen;
  }
  int speedDiff = offset * sensitivity;
  leftDrive(baseSpeed - speedDiff);
  rightDrive(baseSpeed + speedDiff);
}

void strongLineFollow(int targetLastSeen, int sensitivity, int baseSpeed) {

  static bool isRightSide = lastSeen > 3;
  int offset = 0;
  
  if (lastSeen == -1) {
    if (targetLastSeen == 7 || isRightSide) {
      offset = -2;
    }
    else {
      offset = 2;
    }
  }
  else {
    offset = targetLastSeen - lastSeen;
    isRightSide = lastSeen > 3;
  }

  int speedDiff = offset * sensitivity;

  leftDrive(baseSpeed - speedDiff);
  rightDrive(baseSpeed + speedDiff);
}

void leftLineFollow(int targetFirstSeen, 
  int sensitivity, int baseSpeed = 100) {

  static int previousFirstSeen = firstSeen;
  int offset = 0;
  
  if (firstSeen != -1) {
    offset = targetFirstSeen - firstSeen;
    previousFirstSeen = firstSeen;
  }
  else {
    offset = 0;
  }

  int speedDiff = offset * sensitivity;

  leftDrive(baseSpeed - speedDiff);
  rightDrive(baseSpeed + speedDiff);

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
  driveBesideWall(750, 220, 10);
  return delayState(1000);
}

// Wall follow past the two first rocks. This state follows the wall closer
// than the previous state because we do not want to kick the rocks.
bool wallFollowPastRocks() {

  static bool rockSensorStatus = false;
  static int statusChangeCount = 0;

  driveBesideWall(110, 250, 5);
  
  // If the status of the sensor has changed, increment the count.
  if ((backSensorValue < 700 && !rockSensorStatus) || 
      (backSensorValue > 850 && rockSensorStatus)) {
    statusChangeCount++;
    rockSensorStatus = !rockSensorStatus;
  }
  printVal = backSensorValue;
  // Rock, no rock, rock -> 3 status changes -> we move to the next
  // state.
  if (statusChangeCount == 3) {
    // Reset static variables for possible next time.
    rockSensorStatus = false;
    statusChangeCount = 0;
    return true;
  }
  return false;
}

bool turnToFindLine() {
  leftDrive(0);
  rightDrive(200);
  printVal = lastSeen;
  if (digitalRead(LINE_SENSOR[7]) == 1) {
    return true;
  }
  return false;
}

bool clearCorner() {
  leftDrive(180);
  rightDrive(180);
  return delayState(170);
}

bool lineUpForLineFollow() {

  leftDrive(180);
  rightDrive(0);
  printVal = rightWallSensorValue;
  if (lastSeen < 4 && lastSeen > 0) {
    return true;
  }
  return false;
}

bool scoopRight1DeliverRight1(){
  weakLineFollow(2, 20, 140);
  if (rightWallSensorValue > 800) {
    rightDispenser.write(R_DISPENSER_KICK);
  }
  if (rightBarrelSensorValue < 500) {
    rightDispenser.write(R_DISPENSER_KICK);
    rightScoop.write(R_SCOOP_DOWN);
  } 
  return delayState(1000);
}

bool scoopLeft1ResetRightServos() {
  //line follow to the barrel
  strongLineFollow(7, 30, 180);
  rightScoop.write(R_SCOOP_UP);
  printVal = leftBarrelSensorValue;
  if (leftBarrelSensorValue < 900) {
    leftScoop.write(L_SCOOP_DOWN);
    leftDispenser.write(L_DISPENSER_WIND_UP);
    rightDispenser.write(R_DISPENSER_WIND_UP);
    return true;
  }

  return false;
}

bool lineFollowToRight2() {
  strongLineFollow(4, 30, 180);
  if (rightBarrelSensorValue < 700) {
    rightScoop.write(R_SCOOP_DOWN);
  }

  printVal = rightWallSensorValue;
  return amountSeen > 3;
}

bool findMartinique() {
  leftDrive(180);
  rightDrive(180);
  if (rightBarrelSensorValue < 800) {
    rightScoop.write(R_SCOOP_DOWN);
  }
  printVal = rightWallSensorValue;
  return rightWallSensorValue < 300;
}

bool wallFollowScoopRight2() {
  driveBesideWall(300, 180, 15);
  if (rightBarrelSensorValue < 800) {
    rightScoop.write(R_SCOOP_DOWN);
  }

  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 100 && backSensorValue > 500) {
    return true;
  }
  return false;
}

bool turnBackwardDeliverRight2() {
  leftDrive(-180);
  rightDrive(0);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 150) {
    rightDispenser.write(R_DISPENSER_KICK);
    return true;
  }
  return false;
}

bool turnAwayFromWall() {
  leftDrive(-180);
  rightDrive(0);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 680) {
    return true;
  }
  return false;
}

bool wallFollowDeliverLeft1() {
  driveBesideWall(450, 180, 10);
  printVal = backSensorValue;
  if (backSensorValue < 80) {
    leftDispenser.write(L_DISPENSER_KICK);
    rightScoop.write(R_SCOOP_UP);
    rightDispenser.write(R_DISPENSER_WIND_UP);
    return true;
  }
  return false;
}

bool wallFollowToCorner() {
  driveBesideWall(450, 160, 10);
  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 170) {
    return true;
  }
  return false;
}

bool lineUpForRightScoop3() {
  leftDrive(-180);
  rightDrive(0);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue > 315) {
    return true;
  }
  return false;
}

bool passCornerDeliverRight3() {
  leftDrive(180);
  rightDrive(180);

  if (rightBarrelSensorValue < 850) {
    rightScoop.write(R_SCOOP_DOWN);
  }

  if (delayState(300)) {
    rightDispenser.write(R_DISPENSER_KICK);
    return true;
  }
  return false;
}

bool passCornerScoopRight3() {
  leftDrive(180);
  rightDrive(180);
  
  if (rightBarrelSensorValue < 850) {
    rightScoop.write(R_SCOOP_DOWN);
    rightDispenser.write(R_DISPENSER_KICK);
  }
  return delayState(200);
}

bool skimSouthWall() {
  leftDrive(180);
  rightDrive(180);

  printVal = rightWallSensorValue;
  if (rightWallSensorValue < 400) {
    return true;
  }
  return false;
}

bool findSouthLine() {
  leftDrive(0);
  rightDrive(180);

  printVal = lastSeen;
  if (lastSeen != -1) {
    leftScoop.write(L_SCOOP_UP);
    leftDispenser.write(L_DISPENSER_WIND_UP);
    return true;
  }
  return false;
}

bool alignWithSouthLine() {
  strongLineFollow(3, 30, 180);
  delayState(300);
}

bool scoopLeft2DeliverLeft2() {
  strongLineFollow(6, 40, 180);
  
  printVal = backSensorValue;
  if (backSensorValue < 80) {
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
  weakLineFollow(6, 30, 80);
  return delayState(350);
}

bool lineFollowToIsland() {
  weakLineFollow(3, 30, 170);
  if (rightBarrelSensorValue < 800) {
    rightScoop.write(R_SCOOP_DOWN);
  }
  printVal = rightWallSensorValue;
  return rightWallSensorValue < 500;
}

bool scoopRight4DeliverRight4() {
  weakLineFollow(2, 20, 170);

  if (rightBarrelSensorValue < 800) {
    rightScoop.write(R_SCOOP_DOWN);
  }

  printVal = rightWallSensorValue;
  return rightWallSensorValue > 800;
}

bool allowRight4Ejection() {
  weakLineFollow(2, 10, 90);
  rightDispenser.write(R_DISPENSER_KICK);

  return delayState(300);
}

bool blindLeftTurn() {
  leftDrive(0);
  rightDrive(180);

  return delayState(700);
}

bool getLost() {
  leftDrive(180);
  rightDrive(180);
  return delayState(700);
}

bool returnStraight() {
  leftDrive(200);
  rightDrive(0);

  return delayState(880);
}

bool passRocks() {
  leftDrive(250);
  rightDrive(210);
  return delayState(2600);
}

bool blindRightTurn() {
  leftDrive(180);
  rightDrive(0);

  return delayState(800) || (lastSeen <= 4 && lastSeen > -1);
}

bool findLastLine() {
  leftDrive(180);
  rightDrive(180);
  return lastSeen == 4;
}

bool secureLine() {
  
  strongLineFollow(4, 45, 150);

  return delayState(1000);
}

bool findTapeCorner() {
  strongLineFollow(6, 25, 180);

  return amountSeen > 3;
}

bool rotateDelay() {
  leftDrive(-120);
  rightDrive(120);
  return delayState(600);
}

bool rotateOffLine() {
  leftDrive(-120);
  rightDrive(120);
  return lastSeen == 5;
}

bool wallFollowSpain() {
  weakLineFollow(5, 15, 180);
  printVal = rightWallSensorValue;
  return rightWallSensorValue > 900;
}


bool faceSpain() {
  leftDrive(0);
  rightDrive(-150);
  return lastSeen == 7;
}

bool chargeSpain() {
  leftDrive(240);
  rightDrive(180);

  return delayState(200);
}

bool sinkSpain() {
  leftDrive(200);
  rightDrive(200);

  dumper.write(DUMP_UP);

  return delayState(800);
}

bool dance() {
  leftDrive(50);
  rightDrive(50);
  dumper.write(DUMP_DOWN);
  delay(300);
  dumper.write(DUMP_UP);
  delay(600);
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
    case 0: if (start()) state++; break;
    case 1: if (findNorthWall()) state++; break;
    case 2: if (farWallFollow()) state++; break;
    case 3: if (wallFollowPastRocks()) state++; break;
    case 4: if (turnToFindLine()) state++; break;
    case 5: if (clearCorner()) state++; break;
    case 6: if (lineUpForLineFollow()) state++; break;
    case 7: if (scoopRight1DeliverRight1()) state++; break;
    case 8: if (scoopLeft1ResetRightServos()) state++; break;
    case 9: if (lineFollowToRight2()) state++; break;
    case 10: if (findMartinique()) state++; break;
    case 11: if (wallFollowScoopRight2()) state++; break;
    case 12: if (turnBackwardDeliverRight2()) state++; break;
    case 13: if (turnAwayFromWall()) state++; break;
    case 14: if (wallFollowDeliverLeft1()) state++; break;
    case 15: if (wallFollowToCorner()) state++; break;
    case 16: if (lineUpForRightScoop3()) state++; break;
    case 17: if (passCornerDeliverRight3()) state++; break;
    case 18: if (passCornerScoopRight3()) state++; break;
    case 19: if (skimSouthWall()) state++; break;
    case 20: if (findSouthLine()) state++; break;
    case 21: if (alignWithSouthLine()) state++; break;
    case 22: if (scoopLeft2DeliverLeft2()) state++; break;
    case 23: if (allowLeftBarrelEjection2()) state++; break;
    case 24: if (lineFollowToIsland()) state++; break;
    case 25: if (scoopRight4DeliverRight4()) state++; break;
    case 26: if (allowRight4Ejection()) state++; break;
    case 27: if (blindLeftTurn()) state++; break;
    case 28: if (getLost()) state++; break;
    case 29: if (returnStraight()) state++; break;
    case 30: if (passRocks()) state++; break;
    case 31: if (blindRightTurn()) state++; break;
    case 32: if (findLastLine()) state++; break;
    case 33: if (secureLine()) state++; break;
    case 34: if (findTapeCorner()) state++; break;
    case 35: if (rotateDelay()) state++; break;
    case 36: if (rotateOffLine()) state++; break;
    case 37: if (wallFollowSpain()) state++; break;
    case 38: if (faceSpain()) state++; break;
    case 39: if (chargeSpain()) state++; break;
    case 40: if (sinkSpain()) state++; break;
    case 41: if (dance()) state++; break;
    case 42: if (stop()) state++; break;
    default: break;
  } 

  if(iterationCount % 200 == 0 || lastState != state){
    Serial3.print(state);
    Serial3.print(" - ");
    Serial3.println(printVal);
    lastState = state;
  }
  iterationCount++;
}
