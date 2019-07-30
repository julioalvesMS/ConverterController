#ifndef SRC_CORE_SENSOR_H_
#define SRC_CORE_SENSOR_H_

#include "F28x_Project.h"
#include "DPlib.h"

#include <src/Util/Common/constants.h>
#include <src/HAL/ADC/adc.h>
#include <src/HAL/PWM/pwm.h>

#define VOUT_MAX    6.09
#define VIN_MAX     12.09
#define IL_MAX      6.83

#define READ_IL(X) ADCDRV_1ch_F_C(X) * IL_MAX
#define READ_VOUT(X) ADCDRV_1ch_F_C(X) * VOUT_MAX
#define READ_VIN(X) ADCDRV_1ch_F_C(X) * VIN_MAX

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
    double* GetState(void);

    //
    //  GetState - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void);
}

#endif /* SRC_CORE_SENSOR_H_ */
