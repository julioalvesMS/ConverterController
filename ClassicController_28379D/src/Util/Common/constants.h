#ifndef SRC_COMMON_CONSTANTS_H_
#define SRC_COMMON_CONSTANTS_H_

#define CPU_SYS_CLOCK 200000

#define MAIN_PERIOD 1.5

#define SWITCHING_FREQUENCY 200
#define SWITCH_PWM_PERIOD (CPU_SYS_CLOCK)/SWITCHING_FREQUENCY
#define SWITCH_PWM_TBPRD 128

#define ADC_PERIOD CPU_SYS_CLOCK >> 11
#define PERIOD_UNIT 1e3

#define ADC_CONVERSIONS 3


//
// Circuit specifications
//

//  [Ohm] - Converter Resistance
const double R  = 1e-3;

//  [H] - Converter Inductance
const double L  = 4.8e-6;

//  [Ohm] - Load Resistance
const double Ro = 7;

//  [F] - Output Capacitance
const double Co = 726e-6;

//  [Ohm] - Capacitor Resistance
const double Rc = 0.00015;


#endif /* SRC_COMMON_CONSTANTS_H_ */
