#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Servos
Servo leftScoop, rightScoop, rightDispenser, leftDispenser, dumper;

void leftDrive(int speed) {
  int bias = 15;
  if (speed >= 0) {
    digitalWrite(WHEEL_DIR_LF, HIGH);
    digitalWrite(WHEEL_DIR_LB, LOW);
    analogWrite(WHEEL_SPEED_L, speed + bias);
  }
  else {
    digitalWrite(WHEEL_DIR_LF, LOW);
    digitalWrite(WHEEL_DIR_LB, HIGH);
    analogWrite(WHEEL_SPEED_L, -speed + bias);    
  }
}

void rightDrive(int speed) {
  if (speed >= 0) {
    digitalWrite(WHEEL_DIR_RF, HIGH);
    digitalWrite(WHEEL_DIR_RB, LOW);
    analogWrite(WHEEL_SPEED_R, speed);
  }
  else {
    digitalWrite(WHEEL_DIR_RF, LOW);
    digitalWrite(WHEEL_DIR_RB, HIGH);
    analogWrite(WHEEL_SPEED_R, -speed);    
  }
}

bool leftCollect(){
  static int startTime = -1;
  if(startTime == -1){
    startTime = millis();
    leftScoop.write(L_SCOOP_DOWN);
  }
  else if(millis() - startTime > 500){
    leftScoop.write(L_SCOOP_UP);
    startTime = -1;
    return true;
  }
  return false;
}

bool rightCollect(){
  static int startTime = -1;
  if(startTime == -1){
    startTime = millis();
    rightScoop.write(R_SCOOP_DOWN);
  }
  else if(millis() - startTime > 500){
    rightScoop.write(R_SCOOP_UP);
    startTime = -1;
    return true;
  }
  return false;
}

bool dump() {
  static int startTime = -1;
  if(startTime == -1){
    startTime = millis();
    dumper.write(DUMP_UP);
  }
  else if(millis() - startTime > 700){
    dumper.write(DUMP_DOWN);
    startTime = -1;
    return true;
  }
  return false;
}

bool rightDispense(){
  static int startTime = -1;
  if(startTime == -1){
    startTime = millis();
    rightDispenser.write(R_DISPENSER_OUT);
  }
  else if(millis() - startTime > 500){
    rightDispenser.write(R_DISPENSER_IN);
    startTime = -1;
    return true;
  }
  return false;
}

bool leftDispense(){
  static int startTime = -1;
  if(startTime == -1){
    startTime = millis();
    leftDispenser.write(L_DISPENSER_OUT);
  }
  else if(millis() - startTime > 500){
    leftDispenser.write(L_DISPENSER_IN);
    startTime = -1;
    return true;
  }
  return false;
}

bool delayState(int amount) {
  static int startTime = -1;
  if (startTime == -1) {
    startTime = millis();
  }
  else if (millis() - startTime > amount) {
    startTime = -1;
    return true;
  }
  return false;
}

void resetRobot() {  
  leftScoop.write(L_SCOOP_UP);
  rightScoop.write(R_SCOOP_UP);
  dumper.write(DUMP_DOWN);
  rightDispenser.write(R_DISPENSER_IN);
  leftDispenser.write(L_DISPENSER_IN);
  rightDrive(0);
  leftDrive(0);
}

bool start() {
  if (digitalRead(BUTTON1) == LOW) {
    return true;
  }
  else {
    resetRobot();
    return false;
  }
}

bool button2() {
  if (digitalRead(BUTTON2) == LOW) {
    return true;
  }
  return false;
}

bool initialTurn()
{
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

bool wallFind() {
  leftDrive(90);
  rightDrive(90);
  if (analogRead(A0) < 780) {
    return true;
  }
  return false;
}

void driveBesideWall(int targetDistance) {
  int wallSensor = analogRead(A0);
  int offset = wallSensor - targetDistance;
  int baseSpeed = 90;
  int speedDiff = offset / 8;

  leftDrive(baseSpeed + speedDiff);
  rightDrive(baseSpeed - speedDiff);
}

bool farWallFollow() {

  static int startTime = -1;

  if (startTime == -1) {
    startTime = millis();
  }

  if (millis() - startTime > 2500) {
    return true;
  }

  driveBesideWall(780);
  return false;
}

bool closeWallFollow() {
  driveBesideWall(500);
  if(analogRead(A0) < 150) {
    return true;
  }
  return false;
}

bool cornerReverse() {
  leftDrive(-60);
  rightDrive(0);

  if (analogRead(A0) > 730) {
    return true;
  }
  return false;
}

bool lineFind() {
  for (int i = 0; i < 8; i++) {
    if (digitalRead(LINE_SENSOR[i]) == 1) {
      return true;
    }
  }

  leftDrive(70);
  rightDrive(70);

  return false;
}

bool lineFollow(){
  
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

  int baseSpeed = 75;
  int offset = 7 - (firstSeen + lastSeen);
  Serial.println(offset);
  int speedDiff = 0;

  if(firstSeen == -1) speedDiff = 8;
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

  if (button2()) state = 0;

  // State succession
  switch (state) {
    case 0: if (start()) state++; break;
    case 1: if (initialTurn()) state++; break;
    case 2: if (wallFind()) state++; break;
    case 3: if (farWallFollow()) state++; break;
    case 4: if (closeWallFollow()) state++; break;
    case 5: if (cornerReverse()) state++; break;
    case 6: if (lineFind()) state++; break;
    case 7: if (lineFollow()) state++; break;
    default: state = 0;
  }
}
