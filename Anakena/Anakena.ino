//Updated 1-28-17
#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Tracks the current state of the robot.
int state = 0;
int stateStartTime = 0;
int rightCollectStartTime = -1;
int leftCollectStartTime = -1;

//Constants and Motors
Servo leftArm, rightArm, rightDispenser, leftDispenser, dumper;

void setup(){

  //set the pinmodes
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);
  
  //turn LED's on
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDY, HIGH);
  
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  
  pinMode(LEFT_ARM, OUTPUT);
  pinMode(RIGHT_ARM, OUTPUT);
  pinMode(DUMPER, OUTPUT);
  pinMode(RIGHT_DISPENSER, OUTPUT);
  pinMode(LEFT_DISPENSER, OUTPUT);

  //attatch the motors
  leftArm.attach(LEFT_ARM);
  rightArm.attach(RIGHT_ARM);
  rightDispenser.attach(RIGHT_DISPENSER);
  leftDispenser.attach(LEFT_DISPENSER);
  dumper.attach(DUMPER);

  //put all the servos in their place
  leftArm.write(L_ARM_UP);
  rightArm.write(R_ARM_UP);
  dumper.write(DUMP_DOWN);
  rightDispenser.write(RIGHT_DISPENSER_IN_POSITION);
  leftDispenser.write(LEFT_DISPENSER_IN_POSITION);
  
  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR(i), INPUT);
  }

  //================ WHEELS ===============
  // left
  pinMode(WHEEL_PWM_L, OUTPUT);
  analogWrite(WHEEL_PWM_L, 0);

  pinMode(WHEEL_DIR_L_F, OUTPUT);
  pinMode(WHEEL_DIR_L_B, OUTPUT);

  digitalWrite(WHEEL_DIR_L_F, HIGH);
  digitalWrite(WHEEL_DIR_L_B, LOW);
  

  // right
  pinMode(WHEEL_PWM_R, OUTPUT);
  analogWrite(WHEEL_PWM_R, 0);

  pinMode(WHEEL_DIR_R_F, OUTPUT);
  pinMode(WHEEL_DIR_R_B, OUTPUT);

  digitalWrite(WHEEL_DIR_R_F, HIGH);
  digitalWrite(WHEEL_DIR_R_B, LOW);

  // Standby pins
  pinMode(WHEEL_PWM_FIRST_STBY, OUTPUT);
  digitalWrite(WHEEL_PWM_FIRST_STBY, HIGH);
  
  pinMode(WHEEL_PWM_SECOND_STBY, OUTPUT);
  digitalWrite(WHEEL_PWM_SECOND_STBY, HIGH);

  Serial.begin(115200);
  
}

bool hasRun = false;



//==========================================
// LOOP
//==========================================
// The direction of the wheels is determined by two pins, WHEEL_DIR_xx_F and WHEEL_DIR_xx_B.
// F and B stand for Forward and Backward, but that is an oversimplification that is useful
// for remembering which pin is which. The wheels can do four things: Coast, Go forward, Go backward, and Brake. 
// The following is a truth table for the pins:
// F B | Direction
// 0 0 | Coast
// 0 1 | Backwards
// 1 0 | Forward
// 1 1 | 
void loop() {
  switch (state) {
    case START:
      if(digitalRead(BUTTON1) == LOW) {
        goToState(LINE_FOLLOW);
      }
    break;
    case LINE_FOLLOW:
      lineFollow();
    break;
  }
  /*if(!hasRun){
    //lowerDump();
    leftCollect();
    delay(1000);
    rightCollect();
    delay(1000);
    dump();
    delay(1000);
    rightDispense();
    delay(1000);
    leftDispense();
    delay(1000);
  }*/
}

void leftCollect(){
  if(leftCollectStartTime == -1){
    leftCollectStartTime = millis();
    leftArm.write(L_ARM_DOWN);
  }
  else if(millis() + leftCollectStartTime > 500){
    leftArm.write(L_ARM_UP);
    leftCollectStartTime = -1;
  }
}

void rightCollect(){
    if(rightCollectStartTime == -1){
    rightCollectStartTime = millis();
    rightArm.write(R_ARM_DOWN);
  }
  else if(millis() + rightCollectStartTime > 500){
    rightArm.write(R_ARM_UP);
    rightCollectStartTime = -1;
  }
}

void dump(){
  dumper.write(DUMP_UP);
  delay(200);
  dumper.write(DUMP_DOWN);
  //delay(200);
  //dumper.write(DUMP_UP);
}

void lowerDump(){
  dumper.write(DUMP_DOWN);
}

void rightDispense(){
  rightDispenser.write(RIGHT_DISPENSER_OUT_POSITION);
  delay(700);
  rightDispenser.write(RIGHT_DISPENSER_IN_POSITION);
}

void leftDispense(){
  leftDispenser.write(LEFT_DISPENSER_OUT_POSITION);
  delay(700);
  leftDispenser.write(LEFT_DISPENSER_IN_POSITION);
}

void goToState(int targetState) {
  state = targetState;
  stateStartTime = millis();
}

void lineFollow() {
//  int firstSeen = -1;
//  int lastSeen = 0;
  
  for (int i = 0; i < 8; i++) {
    int pin = LINE_SENSOR(i);
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.print(": ");
    int reading = digitalRead(pin);
    Serial.println(reading);
//    if (reading == 1) {
//      if (firstSeen == -1) {
//        firstSeen = i;
//      }
//      lastSeen = i;
//    }
  }
  Serial.println();
//  Serial.println(firstSeen);
//  Serial.println(lastSeen);
//  Serial.println();
  delay(200);
}

void leftDrive(int speed) {
  digitalWrite(WHEEL_DIR_L_F, HIGH);
  digitalWrite(WHEEL_DIR_L_B, LOW);
  analogWrite(WHEEL_PWM_L, speed);
}

void rightDrive(int speed) {
  digitalWrite(WHEEL_DIR_R_F, HIGH);
  digitalWrite(WHEEL_DIR_R_B, LOW);
  analogWrite(WHEEL_PWM_R, speed);
}
