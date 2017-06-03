#include <Servo.h>

// Pins
const int L_SCOOP = 13;
const int R_SCOOP = 12;
const int DUMPER = 11;
const int R_DISPENSER = 10;
const int L_DISPENSER = 9;

// Scoop Angles
const int L_SCOOP_UP   = 160;
const int L_SCOOP_DOWN = 80;
const int R_SCOOP_UP   = 20;
const int R_SCOOP_DOWN = 90;

// Dumper Angles
const int DUMP_DOWN = 83;
const int DUMP_UP   = 125;

// Dispenser Angles
const int R_DISPENSER_OUT = 1;
const int R_DISPENSER_IN = 25;
const int L_DISPENSER_OUT = 20;
const int L_DISPENSER_IN = 2;

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

void setup() {
  // Attach servos
  leftScoop.attach(L_SCOOP);
  rightScoop.attach(R_SCOOP);
  rightDispenser.attach(R_DISPENSER);
  leftDispenser.attach(L_DISPENSER);
  dumper.attach(DUMPER);
}

void loop() {
  static int state = 0;
  switch (state) {
    case 0: if (leftCollect()) state++; 
    break;
    case 1: if (delay(500)) state++;
    break;
    case 2: if (rightCollect()) state++;    
    break;
    case 3: if (delay(500)) state++;
    break;
    case 4: if (dump()) state++;
    break;
    case 5: if (delay(500)) state++;
    break;
    case 6: if (rightDispense()) state++;
    break;
    case 7: if (delay(500)) state++;
    break;
    case 8: if (leftDispense()) state++;
    break;
    case 9: if (delay(500)) state = 0;
    break;
  }
}
