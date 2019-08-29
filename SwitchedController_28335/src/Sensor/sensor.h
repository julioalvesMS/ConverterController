#ifndef SRC_SENSOR_SENSOR_H_
#define SRC_SENSOR_SENSOR_H_

#include "DSP28x_Project.h"

#include <src/settings.h>
#include <src/Common/constants.h>
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/SPI_DAC.h>

namespace Sensor
{
    //
    // VALOR MÉDIO DA TENSÃO DE SAÍDA
    //
    static double vout_mean_buffer[ADC_BUFFER_SIZE];
    static int buffer_index = 0;

    //
    // VARIÁVEIS COM OS VALORES DOS SENSORES
    //
    static double s_state[SYSTEM_ORDER]; // X - State vector
    static double input_voltage = 0;     // u - Input Voltage
    static double vout_mean = 0;        // Vout - Output mean value

    void Configure(void);

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


    __interrupt
    void Interruption(void);
}

#endif /* SRC_SENSOR_SENSOR_H_ */
