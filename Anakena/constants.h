#pragma once

// Arm servo angles.
const int L_ARM_UP   = 160; //CHANGED
const int L_ARM_DOWN = 80;  //CHANGED
const int R_ARM_UP   = 20;  //CHANGED
const int R_ARM_DOWN = 90;  //CHANGED

// Dumper angles
const int DUMP_DOWN = 83;
const int DUMP_UP   = 125;
const int RIGHT_DISPENSER_OUT_POSITION = 1;  //CHANGED
const int RIGHT_DISPENSER_IN_POSITION = 25; //CHANGED
const int LEFT_DISPENSER_OUT_POSITION = 20; //CHANGED
const int LEFT_DISPENSER_IN_POSITION = 2;  //CHANGED

// Speed levels
const int FULL_SPEED = 100;
const int HALF_SPEED = 50;
const int SLOW_SPEED = 10;

// Sensor threshold
const int CLAW_SENSOR_THRESHOLD = 100;
const int BACKWALL_SENSOR_THRESHOLD = 100;
const int ISLAND_SENSOR_THRESHOLD = 100;
const int FRONTWALL_SENSOR_THRESHOLD = 100;

enum State {
    START = 0,
    LINE_FOLLOW = 1
};
