#ifndef SETTINGS_CPU_01_H_
#define SETTINGS_CPU_01_H_


#define REFERENCE_UPDATE_ENABLED 1
#define REFERENCE_CONTROLLER_PERIOD 1000    // Period in Microseconds

#define REFERENCE_CONTROLLER_PID_KP 0.5
#define REFERENCE_CONTROLLER_PID_KI 1

#define GPIO_S1    6
#define GPIO_S2    7
#define GPIO_A1    8


#define FILTERED_IL_SENSOR 0

#define VOUT_MAX    6.09
#define VIN_MAX     12.09
#define IL_MAX      6.83

#define READ_IL(X) ADCDRV_1ch_F_C(X) * IL_MAX
#define READ_VOUT(X) ADCDRV_1ch_F_C(X) * VOUT_MAX
#define READ_VIN(X) ADCDRV_1ch_F_C(X) * VIN_MAX

#define ADC_BUFFER_SIZE     64

#endif /* SETTINGS_CPU_01_H_ */
