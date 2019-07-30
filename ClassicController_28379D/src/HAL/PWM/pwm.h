#ifndef SRC_HAL_PWM_H_
#define SRC_HAL_PWM_H_

#include "F28x_Project.h"
#include <src/Util/Common/constants.h>

namespace PWM
{
    //
    // Configure - Configure EPWM SOC and compare values
    //
    void ADC_Configure(void);

    //
    // Configure - Configure EPWM SOC and compare values
    //
    void Switch_Configure(void);

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC(void);

    //
    // ADC_Start - Start ePWM
    //
    void ADC_Start(void);

    void Switch_UpdateDutyCycle(double DutyCycle);
}

#endif /* SRC_HAL_PWM_H_ */
