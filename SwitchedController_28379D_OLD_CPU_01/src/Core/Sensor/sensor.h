#ifndef SRC_CORE_SENSOR_H_
#define SRC_CORE_SENSOR_H_

#include "F28x_Project.h"
#include "DPlib.h"

#include <src/Util/Common/constants.h>
#include <src/HAL/ADC/adc.h>
#include <src/HAL/PWM/pwm.h>

namespace Sensor
{
    //
    // Configure - Make configurations and initializations needed
    //              to read the sensors
    //
    void Configure(void);

    //
    //  Start - Start Sensor aquisition
    //
    void Start(void);

    //
    //  GetState - Get the pointer to the variable where the state
    //              vector is stored
    //
    double* GetState(void);

    //
    //  GetState - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void);

    //
    //  GetOutput - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetOutput(void);
}

#endif /* SRC_CORE_SENSOR_H_ */
