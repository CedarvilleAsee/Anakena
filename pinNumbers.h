#ifndef PIN_NUM

#define PIN_NUM

#define DRIVE_WHEEL_L     34
#define DRIVE_WHEEL_R     36 

#define LEDY              38
#define LEDG              40
   
#define WHEEL_DIR_FL_B     35
#define WHEEL_DIR_FL_F     37

// pins of line sensors
#define LINE_SENSOR_0 39
#define LINE_SENSOR_1     41
#define LINE_SENSOR_2     43
#define LINE_SENSOR_3     45  
#define LINE_SENSOR_4     47
#define LINE_SENSOR_5     49
#define LINE_SENSOR_6     51
#define LINE_SENSOR_7     520

//start button?
#define BUTTON1           30
#define BUTTON2           32

//Motor Controller Pins
#define WHEEL_DIR_FR_B     34
#define WHEEL_DIR_FR_F     36
#define WHEEL_DIR_FL_B     35
#define WHEEL_DIR_FL_F     37
#define WHEEL_DIR_BR_B     26
#define WHEEL_DIR_BR_F     28
#define WHEEL_DIR_BL_B     27
#define WHEEL_DIR_BL_F     29

// Speed Pins
#define WHEEL_PWM_FR      8
#define WHEEL_PWM_FL      2
#define WHEEL_PWM_BR      7
#define WHEEL_PWM_BL      3

// Standby Pins
// It doesn't matter which standbys control which wheels because all the 
// wheels we be doing the same thing all the time (this is the one robot team).
#define WHEEL_PWM_FIRST_STBY  31
#define WHEEL_PWM_SECOND_STBY  33

// Arm pins
#define LEFT_ARM  13 //CHANGED
#define RIGHT_ARM 12 //CHANGED
#define DUMPER    11 //CHANGED
#define DISPENSER 10 //CHANGED

// Sensors
#define RIGHT_CLAW_SENSOR A0
#define LEFT_CLAW_SENSOR A1
#define FRONT_SENSOR A2
#define BACK_SENSOR A3

#endif 
