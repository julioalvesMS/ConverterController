#ifndef SRC_HAL_ADC_H_
#define SRC_HAL_ADC_H_

#include "F28x_Project.h"

#include <src/Util/Common/constants.h>

namespace ADC_HAL
{
    static unsigned int* resultDestination[3];

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
    void ConfigureInterruption(unsigned int* variables[]);

    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption_A(void);


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption_B(void);


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption_C(void);
}

#endif /* SRC_HAL_ADC_H_ */
