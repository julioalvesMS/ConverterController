#ifndef SRC_COMMON_CONSTANTS_H_
#define SRC_COMMON_CONSTANTS_H_

#define SYSTEM_ORDER 2
#define SUBSYSTEMS_COUNT 3

#define BOARD_PERIOD 100
#define PERIOD_UNIT 1e6

//
// Sensor calibration
//
#define READ_IL(X)      0.0077*X - 4.1675
#define READ_VOUT(X)    0.0355*X - 16.281
#define READ_VIN(X)     0.021*X - 11.507
#define READ_IOUT(X)    0.001*X - 0.5465

//
// Circuit specifications
//
const double R  = 0.4;    //  [Ohm] - Inductor Resistance
const double L  = 1.954e-3;     //  [H]   - Inductance
const double Ro = 96.8;     //  [Ohm] - Load Resistance
const double Co = 2250e-6;  //  [F]   - Output Capacitance

const double Rratio = R/Ro;

#endif /* SRC_COMMON_CONSTANTS_H_ */
