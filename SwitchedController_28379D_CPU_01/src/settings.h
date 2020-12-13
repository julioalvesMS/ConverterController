#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <src/Config/DEFINES_LP28379D.h>

//
// IPC
//
#define IPC_COMMAND_BUFFER_SIZE         64

//
// SWITCHED CONTROL
//
#define INITIAL_REFERENCE_VOLTAGE   0
#define REFERENCE_CONTROLLER_PERIOD 1000    // Period in Microseconds

//
// ADC
//
// Sensores lado esquerdo da placa
// Ix, Iy e Iz e Icc2
#define Ix      AdccResultRegs.ADCRESULT0
#define Iy      AdcaResultRegs.ADCRESULT2   // ORIGINAL: A3
#define Iz      AdcbResultRegs.ADCRESULT0
#define Icc2    AdcbResultRegs.ADCRESULT1
// Vx, Vy e Vz Vcc2
#define Vx      AdcaResultRegs.ADCRESULT1   // ORIGINAL: A2
#define Vy      AdcaResultRegs.ADCRESULT0
#define Vz      AdccResultRegs.ADCRESULT1
#define Vcc2    AdcbResultRegs.ADCRESULT4
// Sensores lado direito da placa
// Iu, Iv e Iw e Icc1
#define Iu      AdccResultRegs.ADCRESULT4
#define Iv      AdcaResultRegs.ADCRESULT1
#define Iw      AdcbResultRegs.ADCRESULT3
#define Icc1    AdccResultRegs.ADCRESULT2
// Vu, Vz e Vw Vcc1
#define Vu      AdccResultRegs.ADCRESULT3
#define Vv      AdcaResultRegs.ADCRESULT4
#define Vw      AdcbResultRegs.ADCRESULT2
#define Vcc1    AdcaResultRegs.ADCRESULT5

#define ADC_RESULT_IL           Ix
#define ADC_RESULT_IOUT         Iy
#define ADC_RESULT_VIN          Vx
#define ADC_RESULT_VOUT         Vy
#define ADC_BUFFER_SIZE         64


//
// SWITCHES
//
#define SWITCH_SWITCHED_PWM_TBPRD    625
#define S1    GPIO10
#define S2    GPIO11
#define S4    GPIO8
#define S3    GPIO9
#define TST   GPIO7

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
#define PROTECTION_IL_MAX       23
#define PROTECTION_IOUT_MAX     2.5

//
// Switching Frequency
//
#define SYSTEM_EVALUATION_PERIOD 100000     // Period in Microseconds

//
// Deadband
//
#define DEAD_BAND_PERIOD 10     // Period in Microseconds

#endif /* SETTINGS_H_ */
