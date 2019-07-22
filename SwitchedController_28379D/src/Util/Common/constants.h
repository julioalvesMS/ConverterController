#ifndef SRC_COMMON_CONSTANTS_H_
#define SRC_COMMON_CONSTANTS_H_

#define SYSTEM_ORDER 2
#define SUBSYSTEMS_COUNT 2

#define BOARD_PERIOD 100
#define PERIOD_UNIT 1e6

#define ADC_CONVERSIONS 3


//
// Circuit specifications
//

//  [Ohm] - Converter Resistance
const double R  = 0.135;

//  [H] - Converter Inductance
const double L  = 5e-3;

//  [Ohm] - Load Resistance
const double Ro = 96.8;

//  [F] - Output Capacitance
const double Co = 2250e-6;

//  [Ohm] - Capacitor Resistance
const double Rc = 0.015;


#endif /* SRC_COMMON_CONSTANTS_H_ */
