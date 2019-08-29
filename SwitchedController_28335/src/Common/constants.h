#ifndef SRC_COMMON_CONSTANTS_H_
#define SRC_COMMON_CONSTANTS_H_

#define SYSTEM_ORDER 2
#define SUBSYSTEMS_COUNT 2

#define BOARD_PERIOD 100
#define PERIOD_UNIT 1e6

//
// ADC result normalization
//
#define INV_4096 ((float) 0.000244140625)               // ADC bit-resolution constant.
#define ADCDRV_1ch_F_C(num) ((float)(num) * INV_4096)   // Normalizes ADC conversions in range [0.0, 1.0]

//
// Sensors maximum outputs
//
#define VOUT_MAX    6.09
#define VIN_MAX     12.09
#define IL_MAX      6.83

//
// Read Sensor values
//
#define READ_IL(X) ADCDRV_1ch_F_C(X) * IL_MAX
#define READ_VOUT(X) ADCDRV_1ch_F_C(X) * VOUT_MAX
#define READ_VIN(X) ADCDRV_1ch_F_C(X) * VIN_MAX
//
// Circuit specifications
//
const double R  = 0.135;    //  [Ohm] - Inductor Resistance
const double L  = 5e-3;     //  [H]   - Inductance
const double Ro = 96.8;     //  [Ohm] - Load Resistance
const double Co = 2250e-6;  //  [F]   - Output Capacitance

#endif /* SRC_COMMON_CONSTANTS_H_ */
