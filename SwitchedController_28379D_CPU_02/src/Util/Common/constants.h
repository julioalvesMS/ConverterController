#ifndef SRC_COMMON_CONSTANTS_H_
#define SRC_COMMON_CONSTANTS_H_

#define SYSTEM_ORDER 2
#define SUBSYSTEMS_COUNT 2

#define MAIN_PERIOD 2
#define REFERENCE_CONTROLLER_PERIOD 100
#define SWITCH_ON_DELAY 1
#define PERIOD_UNIT 1e6

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
