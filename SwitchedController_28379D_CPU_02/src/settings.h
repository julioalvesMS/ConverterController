#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <src/Config/DEFINES_LP28379D.h>

//
// IPC
//
#define IPC_COMMAND_BUFFER_SIZE         64


//
// SWITCHES
//
#define S1    GPIO10
#define S2    GPIO11
#define S4    GPIO8
#define S3    GPIO9

//
// SWITCHES PWMs
//
#define SWITCH_PWM_TBPRD    1250
#define SET_DUTY_S1(x)      GPIO10
#define SET_DUTY_S2(x)      GPIO11
#define SET_DUTY_S4(x)      GPIO8
#define SET_DUTY_S3(x)      GPIO9


//
// Protections
//
#define PROTECTION_VIN_MAX      75
#define PROTECTION_VOUT_MAX     125
#define PROTECTION_IL_MAX       20
#define PROTECTION_IOUT_MAX     2.5

//
// Switching Frequency
//
#define SYSTEM_EVALUATION_PERIOD 100000     // Period in Microseconds

#endif /* SETTINGS_H_ */
