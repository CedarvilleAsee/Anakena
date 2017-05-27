#ifndef CONSTANTS
#define CONSTANTS

// Arm servo angles.
const int L_ARM_UP   = 160; //CHANGED
const int L_ARM_DOWN = 80;  //CHANGED
const int R_ARM_UP   = 20;  //CHANGED
const int R_ARM_DOWN = 90;  //CHANGED

// Dumper angles
const int DUMP_DOWN = 83;
const int DUMP_UP   = 125;
const int DISPENSE_OUT   = 30;
const int DISPENSE_BACK  = 5;

// Kicker angles
const int KICKER_READY = 15;
const int KICKER_KICKED = 45;

// Dumper angle
const int DUMPER_DUMPED = 45;

// Mousetrap positions
const int MOUSETRAP_DEPLOYED = 45;
const int MOUSETRAP_DEPOSITED = 15;

// Speed levels

const int FULL_SPEED = 100;
const int HALF_SPEED = 50;
const int SLOW_SPEED = 10;

// Sensor threshold
const int CLAW_SENSOR_THRESHOLD = 100;
const int BACKWALL_SENSOR_THRESHOLD = 100;
const int ISLAND_SENSOR_THRESHOLD = 100;
const int FRONTWALL_SENSOR_THRESHOLD = 100;

#endif
