#ifndef SRC_HAL_PWM_H_
#define SRC_HAL_PWM_H_

#include "F28x_Project.h"

namespace PWM
{
    //
    // Configure - Configure EPWM SOC and compare values
    //
    void Configure(void);

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC(void);

    //
    // Start - Start ePWM
    //
    void Start(void);
}

#endif /* SRC_HAL_PWM_H_ */
