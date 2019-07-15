#ifndef SRC_SENSOR_SENSOR_H_
#define SRC_SENSOR_SENSOR_H_

#include "DSP28x_Project.h"
#include "DSP2833x_Device.h"
#include <src/Math/matrix.h>

using namespace Math;

namespace Sensor
{
    void init(void);

    __interrupt void isr_interruption(void);

    Vector* getState(void);

    double* getInput(void);
}

#endif /* SRC_SENSOR_SENSOR_H_ */
