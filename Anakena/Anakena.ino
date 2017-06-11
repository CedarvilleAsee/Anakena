#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Servos
Servo leftScoop, rightScoop, rightDispenser, leftDispenser, dumper;

// Drives the left wheel at a specified speed.
void leftDrive(int speed) {

  // The bias accounts for the extra weight of the batteries on the left side
  // of the robot.
  int bias = 15;

  // Check the sign of the speed. If the speed is negative, we drive the wheel
  // backward.
  if (speed >= 0) {
    // HIGH-LOW combination drives wheel forward.
    digitalWrite(WHEEL_DIR_LF, HIGH);
    digitalWrite(WHEEL_DIR_LB, LOW);
    analogWrite(WHEEL_SPEED_L, speed + bias);
  }
  else {
    // LOW-HIGH combination drives wheel backward.
    digitalWrite(WHEEL_DIR_LF, LOW);
    digitalWrite(WHEEL_DIR_LB, HIGH);
    analogWrite(WHEEL_SPEED_L, -speed + bias);    
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

// Set servos and motors to their initial positions and speeds.
void resetRobot() {
  leftScoop.write(L_SCOOP_UP);
  rightScoop.write(R_SCOOP_UP);
  dumper.write(DUMP_DOWN);
  rightDispenser.write(R_DISPENSER_IN);
  leftDispenser.write(L_DISPENSER_IN);
  rightDrive(0);
  leftDrive(0);
}

// Starting state; makes sure the robot is reset.
bool start() {
  if (digitalRead(BUTTON1) == LOW) {
    return true;
  }
  else {
    resetRobot();
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
void driveBesideWall(int targetDistance) {
  int wallSensor = analogRead(R_WALL_SENSOR);
  int offset = wallSensor - targetDistance;
  int baseSpeed = 90;
  int speedDiff = offset / 8;

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

  if (millis() - startTime > 2500) {
    startTime = -1;
    return true;
  }

  driveBesideWall(780);
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
  bool newStatus = analogRead(BACK_SENSOR) < 300;

  // If the status of the sensor has changed, increment the count.
  if (newStatus != rockSensorStatus) statusChangeCount++;

  // Rock, no rock, rock, no rock -> 4 status changes -> we move to the next
  // state.
  if (statusChangeCount == 4) {
    // Reset static variables for possible next time.
    rockSensorStatus = false;
    statusChangeCount = 0;
    return true;
  }

  driveBesideWall(550);
  return false;
}

// Line follow with a bias toward the right; if not line sensor is triggered,
// we will assume the line is to our right.
bool rightBiasLineFollow(){
  
  // Calculate the first and last sensor which see the line.
  int firstSeen = -1;
  int lastSeen = 8;
  for (int i = 0; i < 8; i++) {
    if (digitalRead(LINE_SENSOR[i])== 1) {
      if (firstSeen == -1) {
        firstSeen = i;
      }
      lastSeen = i;
    }
  }

  // Calculate the robots current offset. We want the first and last sensors to
  // be 3 and 4, so the sum should be 7, meaning the offset should be 0 if we
  // are centered.
  int offset = 7 - (firstSeen + lastSeen);

  int baseSpeed = 75;
  int speedDiff = 0;

  if(firstSeen == -1) speedDiff = -10;
  else speedDiff = offset * 5;

  leftDrive(baseSpeed + offset);
  rightDrive(baseSpeed - offset);
  
  return false;
}

void setup(){

  // LEDs
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);

  // Buttons
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  // Line sensors
  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR[i], INPUT);
  }

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

  // Attach servos
  leftScoop.attach(L_SCOOP);
  rightScoop.attach(R_SCOOP);
  rightDispenser.attach(R_DISPENSER);
  leftDispenser.attach(L_DISPENSER);
  dumper.attach(DUMPER);

  // Initial servo placement
  resetRobot();

  // Initial wheel power
  digitalWrite(WHEEL_DIR_LF, HIGH);
  digitalWrite(WHEEL_DIR_LB, HIGH);
  
  digitalWrite(WHEEL_DIR_RF, HIGH);
  digitalWrite(WHEEL_DIR_RB, HIGH);
  
  digitalWrite(WHEEL_STBY, HIGH);
  
  // Turn  green LED  on
  digitalWrite(LEDG, HIGH);

  Serial.begin(115200);
  
}


void loop() {
  static int state = 0;

  // Button 2 resets the state machine. This is useful as a less violent way to
  // reset the robot then using the power switch.
  if (digitalRead(BUTTON2) == LOW) state = 0;

  // State succession
  switch (state) {
    case 0: if (start()) state++; break;
    case 1: if (initialTurn()) state++; break;
    case 2: if (wallFind()) state++; break;
    case 3: if (farWallFollow()) state++; break;
    case 4: if (wallFollowPastRocks()) state++; break;
    case 5: if (rightBiasLineFollow()) state++; break;
    default: state = 0;
  }
}
