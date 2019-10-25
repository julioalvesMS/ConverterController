#ifndef SRC_SENSOR_SENSOR_H_
#define SRC_SENSOR_SENSOR_H_

#include "F28x_Project.h"

#include <src/settings.h>
#include <src/Common/constants.h>
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/SPI_DAC.h>

namespace Sensor
{
    void Configure(void);


    void ConfigureFrequency(int pwmTBPRD);


    //
    //  GetState - Get the pointer to the variable where the state
    //              vector is stored
    //
    double* GetState(void);

    //
    //  GetInput - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void);

    //
    //  GetOutput - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetOutput(void);

    double* GetOutputCurrent(void);

    void ReadADCResult(void);
}

#endif /* SRC_SENSOR_SENSOR_H_ */
