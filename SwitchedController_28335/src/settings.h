#ifndef SETTINGS_H_
#define SETTINGS_H_

//
// SWITCHED CONTROL
//
#define SWITCHING_RULE 2
#define INITIAL_REFERENCE_VOLTAGE   2

//
// REFERENCE UPDATE CONTROLLER
//
#define REFERENCE_UPDATE_ENABLED    0
#define REFERENCE_CONTROLLER_PERIOD 1000    // Period in Microseconds
#define REFERENCE_CONTROLLER_PID_KP 0.5
#define REFERENCE_CONTROLLER_PID_KI 10

//
// ADC
//
// Sensores lado esquerdo da placa
// Ix, Iy e Iz e Icc2
#define Ix      AdcMirror.ADCRESULT0
#define Iy      AdcMirror.ADCRESULT1
#define Iz      AdcMirror.ADCRESULT2
#define Icc2    AdcMirror.ADCRESULT3
// Vx, Vy e Vz Vcc2
#define Vx      AdcMirror.ADCRESULT4
#define Vy      AdcMirror.ADCRESULT6
#define Vz      AdcMirror.ADCRESULT5
#define Vcc2    AdcMirror.ADCRESULT7
// Sensores lado direito da placa
// Iu, Iv e Iw e Icc1
#define Iu      AdcMirror.ADCRESULT8
#define Iv      AdcMirror.ADCRESULT9
#define Iw      AdcMirror.ADCRESULT12
#define Icc1    AdcMirror.ADCRESULT15
// Vu, Vz e Vw Vcc1
#define Vu      AdcMirror.ADCRESULT10
#define Vv      AdcMirror.ADCRESULT11
#define Vw      AdcMirror.ADCRESULT13
#define Vcc1    AdcMirror.ADCRESULT14

#define ADC_RESULT_IL           Ix
#define ADC_RESULT_VOUT         Vx
#define ADC_RESULT_VIN          Vy
#define ADC_BUFFER_SIZE         64

//
// DAC
//
#define DAC_ENABLED     0

//
// SWITCHS
//
#define S1    GPIO10
#define S2    GPIO11
#define MF    GPIO8
#define AF    GPIO9


#endif /* SETTINGS_H_ */
