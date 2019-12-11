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
    // Start - Start ePWM
    //
    void Start(void);
}

#endif /* SRC_HAL_PWM_H_ */
