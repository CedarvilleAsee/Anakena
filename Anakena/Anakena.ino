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
int firstSeen = 9;
int lastSeen = 8;
int amountSeen = 0;
int p = 0;

/*int digit1, digit2, digit3, digit4;
void writeDigit1(int x) {
  digit1 = x % 10;
}
void writeDigit2(int x) {
  digit2 = x % 10;
}
void writeDigit3(int x) {
  digit3 = x % 10;
}
void writeDigit4(int x) {
  digit4 = x % 10;
}
void writeDigits() {
  static int lastTime = millis();
  if (millis() - lastTime > 100) {
    Serial3.write(0x76);
    int num = 1000 * digit1 + 100 * digit2 + 10 * digit3 + digit4;
    Serial3.print(num);
    lastTime = millis();
  }
}*/

#ifndef MANUAL
// Drives the left wheel at a specified speed.
void leftDrive(int speed) {

  // The bias accounts for the extra weight of the batteries on the left side
  // of the robot.
  int bias = 0;

  // Check the sign of the speed. If the speed is negative, we drive the wheel
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

#else
void leftDrive(int speed) { }
void rightDrive(int speed) { }
#endif

void readSensors(){

  val_R_WALL_SENSOR =   analogRead(R_WALL_SENSOR);
  val_R_BARREL_SENSOR = analogRead(R_BARREL_SENSOR);
  val_L_BARREL_SENSOR = analogRead(L_BARREL_SENSOR);
  val_BACK_SENSOR =     analogRead(BACK_SENSOR);

  //Line Sensor
  // Calculate the first and last sensor which see the line.
  firstSeen = 9;
  lastSeen = 8;
  amountSeen = 0;
  for (int i = 0; i < 8; i++) {
    //Serial.print(digitalRead(LINE_SENSOR[i]));
    //Serial.print(" ");
    if (digitalRead(LINE_SENSOR[i])== 1) {
      if (firstSeen == 9) {
        firstSeen = i;
      }
      lastSeen = i;
      amountSeen++;
    }
  }
  /*Serial.print(" | ");
  Serial.print(firstSeen);
  Serial.print(" ");
  Serial.print(lastSeen);
  Serial.print(" ");
  Serial.println(amountSeen);*/
}


// Set servos and motors to their initial positions and speeds.
void resetRobot() {
  returnScoopers();
  dumper.write(DUMP_DOWN);

  digitalWrite(WHEEL_DIR_LF, LOW);
  digitalWrite(WHEEL_DIR_LB, LOW);
  
  digitalWrite(WHEEL_DIR_RF, LOW);
  digitalWrite(WHEEL_DIR_RB, LOW);
  
  digitalWrite(WHEEL_STBY, HIGH);
}

void returnScoopers(){
  leftScoop.write(L_SCOOP_UP);
  rightScoop.write(R_SCOOP_UP);
  rightDispenser.write(R_DISPENSER_IN);
  leftDispenser.write(L_DISPENSER_IN);
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

  if (millis() - startTime > 2500) {
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
  if ((backSensor < 500 && !rockSensorStatus) || 
      (backSensor > 700 && rockSensorStatus)) {
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

  driveBesideWall(550,90);
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
void lineFollow(){

    leftDrive(FOLLOW_SPEED_L[firstSeen]);
    rightDrive(FOLLOW_SPEED_R[firstSeen]);
    /*Serial3.print(firstSeen);
    Serial3.print(" ");
    Serial3.print(FOLLOW_SPEED_L[firstSeen]);
    Serial3.print(" ");
    Serial3.println(FOLLOW_SPEED_R[firstSeen]);*/
}

void lineFollowOffset(){

    leftDrive(FOLLOW_OFFSET_L[firstSeen]);
    rightDrive(FOLLOW_OFFSET_R[firstSeen]);
    /*Serial3.print(firstSeen);
    Serial3.print(" ");
    Serial3.print(FOLLOW_SPEED_L[firstSeen]);
    Serial3.print(" ");
    Serial3.println(FOLLOW_SPEED_R[firstSeen]);*/
}

bool lineFollowToBarrel(){
  //line follow to the barrel
  lineFollow();
  pv = analogRead(R_BARREL_SENSOR);
  if (analogRead(R_BARREL_SENSOR) < 400) {
    return true;
  } 
  return false;
}

bool lineFollowTillSeeLeftBarrel(){
  //line follow to the barrel
  returnScoopers();
  lineFollowOffset();
  pv = analogRead(L_BARREL_SENSOR);
  if (analogRead(L_BARREL_SENSOR) < 900) {
    return true;
  } 
  return false;
}

bool lineFollowToCorner(){
  //line follow to the barrel
  returnScoopers();
  lineFollow();
  if (amountSeen == 0) {
    return true;
  } 
  return false;
}


bool scoopBarrel1(){
  int startTime=millis();
  //line follow
  lineFollow();
  rightDispenser.write(R_DISPENSER_OUT);
  rightScoop.write(R_SCOOP_DOWN);
  while(millis()-startTime < 500){}
  return true;
}

bool scoopBarrel2(){
  int startTime=millis();
  //line follow
  lineFollow();
  leftScoop.write(L_SCOOP_DOWN);
  while(millis()-startTime < 500){}
  return true;
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

  readSensors();
  static int state = 0;
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
    case 0:   if (start()) state++; break;
    case 1:   if (initialTurn()) state++; break;
    case 2:   if (wallFind()) state++; break;
    case 3:   if (farWallFollow()) state++; break;
    case 4:   if (wallFollowPastRocks()) state++; break;
    case 5:   if (turnToFindLine()) state++; break;
    case 6:   if (clearCorner()) state++; break;
    case 7:   if (lineUpForLineFollow()) state++; break;
    case 8:   if (lineFollowToBarrel()) state++; break;
    case 9:   if (scoopBarrel1()) state++; break;
    case 10:  if (lineFollowTillSeeLeftBarrel()) state++; break;
    case 11:  if (scoopBarrel2()) state++; break;
    case 12:  if (lineFollowToCorner()) state++; break;
    case 13:  if (stop()) state++; break;
    default:  state = 0;
  } 
}
