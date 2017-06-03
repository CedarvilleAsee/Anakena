#pragma once

const int LEDY = 38;
const int LEDG = 40;

// Line Sensors
char LINE_SENSOR(int i) {
  static const char pins[] = {51,49,47,45,43,41,39,37};
  return pins[i];
}

// Buttons
const int BUTTON1 = 30;
const int BUTTON2 = 32;

// Arm Pins
const int LEFT_ARM = 13;
const int RIGHT_ARM = 12;
const int DUMPER = 11;
const int RIGHT_DISPENSER = 10;
const int LEFT_DISPENSER = 9;

//Wheels
const int WHEEL_DIR_L_B  =   26;
const int WHEEL_DIR_L_F  =   28;
const int WHEEL_DIR_R_B  =   34;
const int WHEEL_DIR_R_F  =   36;

// Speed Pins
const int WHEEL_PWM_L   =   7;
const int WHEEL_PWM_R   =   8;

// Standby Pins
const int WHEEL_PWM_FIRST_STBY   = 31;
const int WHEEL_PWM_SECOND_STBY  = 33;

