#ifndef SETTINGS_CPU_01_H_
#define SETTINGS_CPU_01_H_


#define REFERENCE_UPDATE_ENABLED 0

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


#endif /* SETTINGS_CPU_01_H_ */
