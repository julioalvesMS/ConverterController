#ifndef SRC_SENSOR_SENSOR_H_
#define SRC_SENSOR_SENSOR_H_

#include "F28x_Project.h"
#include "DPlib.h"

#include <src/Util/Common/constants.h>
#include <src/Util/Math/matrix.h>
#include <src/HAL/ADC/adc.h>
#include <src/HAL/PWM/pwm.h>

using namespace Math;

namespace Sensor
{
    //
    // Configure - Make configurations and initializations needed
    //              to read the sensors
    //
    void Configure(void);

    void Start(void);

    //
    //  GetState - Get the pointer to the variable where the state
    //              vector is stored
    //
    Vector* GetState(void);

    //
    //  GetState - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void);

    //
    //  UpdateState - Update the state vector with the last read
    //
    void UpdateState(void);

    //
    //  GetState - Update the input voltage with the last read
    //
    void UpdateInput(void);
}

#endif /* SRC_SENSOR_SENSOR_H_ */
