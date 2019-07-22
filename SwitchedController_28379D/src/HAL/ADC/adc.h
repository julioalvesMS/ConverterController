#include "F28x_Project.h"

#include <src/Util/Common/constants.h>

namespace ADC
{
    static double* resultDestination[3];

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
    // ConfigureInterruption - Set the interruption handler and define where
    //                          to send the results from the conversion
    //
    void ConfigureInterruption(double* variables[]);

    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption(void);
}
