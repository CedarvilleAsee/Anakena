#pragma once

// Scoop angles
const int L_SCOOP_UP   = 122;
const int L_SCOOP_DOWN = 85;
const int R_SCOOP_UP   = 75;
const int R_SCOOP_DOWN = 112;
const int maxSpeed = 200;

// Dumper angles
const int DUMP_DOWN = 100;
const int DUMP_UP   = 140;

// Dispenser angles
const int R_DISPENSER_OUT = 1;
const int R_DISPENSER_IN = 25;
const int L_DISPENSER_OUT = 20;
const int L_DISPENSER_IN = 2;

// Speeds for line following
int FOLLOW_SPEED_R[] = { 200, 175, 150, 100, 100, 50, 25, 0 };
int FOLLOW_SPEED_L[] = { 0, 25, 50, 100, 100, 150, 175, 200 };

int FOLLOW_OFFSET_R[] = { 200, 200, 200, 200, 200, 150, 100, 50 };
int FOLLOW_OFFSET_L[] = { 0, 0, 0, 25, 50, 75, 100, 150 };
