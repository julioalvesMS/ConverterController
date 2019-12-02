#ifndef SRC_HAL_ADC_H_
#define SRC_HAL_ADC_H_

#include "F28x_Project.h"
#include "DPlib.h"

#include <src/settings_cpu_01.h>
#include <src/Util/Common/constants.h>
#include <src/HAL/DAC/dac.h>

#define ADC_TRIG_SOURCE         5   // Triggered by PWM4A

#define ADC_CHANNEL_IL          4   // ADC_A4
#define ADC_CHANNEL_VIN         4   // ADC_B4
#define ADC_CHANNEL_VOUT        4   // ADC_C4
#define ADC_CHANNEL_IL_AVG      5   // ADC_C5

#if FILTERED_IL_SENSOR
#define ADC_RESULT_IL           AdccResultRegs.ADCRESULT1
#else
#define ADC_RESULT_IL           AdcaResultRegs.ADCRESULT0
#endif
#define ADC_RESULT_VOUT         AdccResultRegs.ADCRESULT0
#define ADC_RESULT_VIN          AdcbResultRegs.ADCRESULT0


namespace ADC_HAL
{
    static double* resultDestination[3];

    static double *vout_mean;

    //
    // Configure - Write ADC configurations and power up the ADC for both
    //                ADC A and ADC B
    //
    void Configure(void);

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC(void);

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_A(void);

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_B(void);

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_C(void);

    //
    // ConfigureInterruption - Set the interruption handler and define where
    //                          to send the results from the conversion
    //
    void ConfigureInterruption(double* variables[], double* vout_mean_pointer);

    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void Interruption(void);
}

#endif /* SRC_HAL_ADC_H_ */