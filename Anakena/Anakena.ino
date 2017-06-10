#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Servos
Servo leftScoop, rightScoop, rightDispenser, leftDispenser, dumper;

void leftDrive(int speed) {
  if (speed >= 0) {
    digitalWrite(WHEEL_DIR_LF, HIGH);
    digitalWrite(WHEEL_DIR_LB, LOW);
    analogWrite(WHEEL_SPEED_L, speed);
  }
  else {
    digitalWrite(WHEEL_DIR_LF, LOW);
    digitalWrite(WHEEL_DIR_LB, HIGH);
    analogWrite(WHEEL_SPEED_L, -speed);    
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

bool lineFollow_old() {
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

bool initialTurn()
{
  static int startTime = -1;
  leftDrive(175);
  rightDrive(160);
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
  leftDrive(150);
  rightDrive(150);
  if (analogRead(A0) < 750) {
    return true;
  }
  return false;
}

bool wallFollow() {
  int wallSensor = analogRead(A0);
  int offset = (wallSensor - 700) / 4;
  Serial.println(offset);
  if (wallSensor < 80) {
    leftDrive(-70);
    rightDrive(-40);
  }
  else if (offset > 0) {
    leftDrive(130);
    rightDrive(130 - min(offset, 70));
  }
  else if (offset < 0) {
    leftDrive(130 + max(offset, -70));
    rightDrive(130);
  }
  else {
    leftDrive(130);
    rightDrive(130);    
  }
  return false;
}

//read the data from line sensor
void readData() {
  
  // Preserve necessary information

  
}

void lineFollow(){
  
  //variables
  int lineData[8];
  int amountSeen = 0;
  int firstLineIndex = -1;

  // Reads in the line sensor data
  for(int i = 0; i<8; i++){
      lineData[i] = digitalRead(LINE_SENSOR[i]);
  }

  //print the line data to serial
  for(int i = 0; i < 8; i++){
    Serial.print(lineData[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Record the line sensor's data
  for(int i = 0; i < 8; i++) {
    if(lineData[i] == 1) {       
       if(firstLineIndex == -1) {
          firstLineIndex = i;
       }
       amountSeen++;
    }
  }
  
  if(firstLineIndex==-1){
    leftDrive(0);
    rightDrive(0);
  }
  else{
    leftDrive(FOLLOW_SPEED_L[firstLineIndex]);
    rightDrive(FOLLOW_SPEED_L[firstLineIndex]);
  }
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

  //

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
  //digitalWrite(WHEEL_DIR_LF, HIGH);
  //digitalWrite(WHEEL_DIR_LB, HIGH);
    
  //digitalWrite(WHEEL_DIR_RF, HIGH);
  //digitalWrite(WHEEL_DIR_RB, HIGH);
  
  //digitalWrite(WHEEL_STBY, HIGH);
  
  // Turn  green LED  on
  digitalWrite(LEDG, HIGH);

  Serial.begin(115200);
  
}


void loop() {
  static int state = 0;

  //state machine code
  if (button2()){
    state = 0;
  }
  switch (state) {
    case 0: start(); break;
    case 1: lineFollow(); break;
    default: state=0;
  }
}



