//Updated 1-28-17
#include <Servo.h>
//#include <lineFollow.h>
#include "pinNumbers.h"
#include "constants.h"
#include "wheels.h"
#include "arms.h"
#include "centralServos.h"

using namespace wheels;

//Constants and Motors
Servo leftArm, rightArm, dispenser, dumper;

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
  pinMode(DISPENSER, OUTPUT);
  pinMode(DUMPER, OUTPUT);

  //attatch the motors
  leftArm.attach(LEFT_ARM);
  rightArm.attach(RIGHT_ARM);
  dispenser.attach(DISPENSER);
  dumper.attach(DUMPER);
}

void loop() {

  lowerDump();
  leftCollect();
  delay(1000);
  rightCollect();
  delay(1000);
  dump();
  delay(1000);
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
  delay(200);
  dumper.write(DUMP_UP);
}
void lowerDump(){
  dumper.write(DUMP_DOWN);
}

