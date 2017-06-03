//Updated 1-28-17
#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Servos
Servo leftScoop, rightScoop, rightDispenser, leftDispenser, dumper;

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

bool delay(int amount) {
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

void leftDrive(int speed) {
  digitalWrite(WHEEL_DIR_LF, HIGH);
  digitalWrite(WHEEL_DIR_LB, LOW);
  analogWrite(WHEEL_SPEED_L, speed);
}

void rightDrive(int speed) {
  digitalWrite(WHEEL_DIR_RF, HIGH);
  digitalWrite(WHEEL_DIR_RB, LOW);
  analogWrite(WHEEL_SPEED_R, speed);
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

bool lineFollow() {
  int firstSeen = -1;
  int lastSeen = 8;
  
  for (int i = 0; i < 8; i++) {
    if (digitalRead(LINE_SENSOR[i]) == 1) {
      if (firstSeen == -1) {
        firstSeen = i;
      }
      lastSeen = i;
    }
  }
  
  int offset = lastSeen + firstSeen;
  Serial.println(offset);

  if (offset < 7) {
    leftDrive(100);
    rightDrive(OFFSET_SPEED[7 - offset]);
  }
  else if (offset > 7) {
    rightDrive(100);
    leftDrive(OFFSET_SPEED[offset - 7]);
  }
  else {
    leftDrive(100);
    rightDrive(100);
  }
  
  return false;
}

void setup(){

  // LEDs
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);

  // Buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

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
  switch (state) {
    case 0: if (start()) state++; 
    break;
    case 1: if (lineFollow()) state++;
    break;
    case 2: state = 0;
    break;
  }
}


