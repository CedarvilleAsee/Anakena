//Updated 1-28-17
#include <Servo.h>
#include "pinNumbers.h"
#include "constants.h"

// Tracks the current state of the robot.
int state = 0;
int stateStartTime = 0;

//Constants and Motors
Servo leftArm, rightArm, rightDispenser, leftDispenser, dumper;

void setup(){

  //set the pinmodes
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);

  //wheel initialization
//  pinMode(WHEEL_DIR_BR_B, OUTPUT);
//  pinMode(WHEEL_DIR_BR_F, OUTPUT);
//  pinMode(WHEEL_DIR_BL_B, OUTPUT);
//  pinMode(WHEEL_DIR_BL_F, OUTPUT);
//
//  digitalWrite(WHEEL_DIR_BR_B, LOW);
//  digitalWrite(WHEEL_DIR_BR_F, HIGH);
//  digitalWrite(WHEEL_DIR_BL_B, LOW);
//  digitalWrite(WHEEL_DIR_BL_F, HIGH);
  
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
    pinMode(37 + i * 2, INPUT);
  }

  Serial.begin(115200);
  
}

bool hasRun = false;

void goToState(int targetState) {
  state = targetState;
  stateStartTime = millis();
}

void lineFollow() {
  int firstSeen = -1;
  int lastSeen = 0;
  for (int i = 0; i < 8; i++) {
    int pin = 37 + i * 2;
    Serial.println(digitalRead(pin));
    if (digitalRead(pin) == 1) {
      if (firstSeen == -1) {
        firstSeen = i;
      }
      lastSeen = i;
    }
  }
  Serial.println();
  Serial.println(firstSeen);
  Serial.println(lastSeen);
  Serial.println();
  delay(1000);
}

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
  leftArm.write(L_ARM_DOWN);
  delay(500);
  leftArm.write(L_ARM_UP);
}

void rightCollect(){
  rightArm.write(R_ARM_DOWN);
  delay(500);
  rightArm.write(R_ARM_UP);
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
